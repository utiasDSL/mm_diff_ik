#!/usr/bin/env python2

''' Demo to show redundancy resolution between base and arm with force control.
'''

from __future__ import print_function, division

import rospy
import numpy as np
import sys
import tf.transformations
import copy

from geometry_msgs.msg import Vector3, Vector3Stamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
import tf2_geometry_msgs

from force_pid import ForcePID
from utils import AveragingFilter, TransformLookup, bound_array
from logger import Logger

from thing_demos.msg import KeyboardTrigger


INIT_POSE_DURATION = 10.0  # Seconds
MAX_INPUT_FORCE = 10  # Maximum force that can be input to PID controller (N)
FORCE_THRESHOLD = 5  # Force required for force control to be used (N)
CONTACT_FORCE = 5  # Desired contact force (N)
NUM_FILTER_PTS = 50  # TODO super high just to make things nice for demo
TIME_STEP = 0.1  # Seconds TODO we can push this? may fuck up servo commands then


def transform_force(f, tf):
    ''' Transform force vector f using stamped transform tf. Note that the
        translation component is not used. '''
    # f is an np array of length 3
    v = Vector3Stamped()
    v.vector = Vector3(*f)
    f = tf2_geometry_msgs.do_transform_vector3(v, tf).vector
    return np.array([f.x, f.y, f.z])


class HumanInteractionDemo(object):
    def __init__(self):
        self.tf_lookup = TransformLookup()
        self.logger = Logger()

        # Timestep for the main loop.
        self.dt = TIME_STEP

        # Wait a second for the TF lookup to initialize.
        rospy.sleep(1.0)

        # Initialize zero positions.
        self.init_zero_position()

        # Force control.
        self.force_filter = AveragingFilter(NUM_FILTER_PTS)
        self.force = np.zeros(3)
        self.force_pid = ForcePID(Kp=np.zeros(3), Ki=0.02*np.ones(3),
                                  Kd=np.zeros(3), set_point=np.zeros(3),
                                  reverse_output=True)
        # self.force_pid.windup_guard = out_max / 0.01

        # gripper commands
        self.pub_gripper = rospy.Publisher('FRL/remote_trigger',
                                           KeyboardTrigger, queue_size=10)

        # publisher for thing_control node
        self.pub_goal_servo = rospy.Publisher('/servo/command',
                                              Float64MultiArray, queue_size=10)

        # subscriber for FT sensor
        self.sub_FT = rospy.Subscriber('/robotiq_force_torque_wrench_zero',
                                       WrenchStamped, self.ft_callback)

        # zero out FT sensor bias
        rospy.set_param('/FT_sensor_bias_node/set_zero', True)

        self.force_detected = False
        self.gripper_changed_time = 0  # Time when gripper last moved

        self.open_timer = None
        self.loop_timer = None

    def init_zero_position(self):
        ''' Get the initial positions of the arm and base. '''
        self.ee_to_world_tf0 = self.tf_lookup.get('odom', 'thing_tool')
        self.ee_to_base_tf0 = self.tf_lookup.get('ur10_arm_base', 'thing_tool')
        self.base_to_world_tf0 = self.tf_lookup.get('odom', 'base_link')

        # Determine offset between the origin of the base frame and where the
        # arm connects to the base. Note the offset is only in the x-direction,
        # and the offset is in fact negative, because the base frame is *in
        # front* of where the arm attaches.
        arm_base_tf = self.tf_lookup.get('base_link', 'ur10_arm_base')
        self.arm_base_offset = arm_base_tf.translation

        print("EE zero position: ", self.ee_to_world_tf0)

    def ft_callback(self, msg):
        ''' Callback when a force reading is received. '''
        # Pushing on the tip of the end effector results in negative force in
        # the z-direction.
        # msg is a WrenchStamped
        f = msg.wrench.force
        self.force = self.force_filter.filter(np.array([f.x, f.y, f.z]))
        self.logger.force_raw = f

    def loop(self):
        self.open_gripper()
        while not rospy.is_shutdown():
            self.force_control_loop()
            rospy.sleep(self.dt)

    def get_current_ee_tf(self):
        return self.tf_lookup.get('odom', 'thing_tool')  # thing_tool is the way

    def _send_gripper_cmd(self, label):
        # if the gripper has moved too recently, do nothing
        # if rospy.get_time() - self.gripper_changed_time < 1.0:
        #     return
        msg = KeyboardTrigger()
        msg.label = label
        self.pub_gripper.publish(msg)
        self.gripper_changed_time = rospy.get_time()

    def open_gripper(self, event=None):
        self._send_gripper_cmd('o')

    def launch_open_timer(self):
        if self.open_timer:
            self.open_timer.shutdown()
        self.open_timer = rospy.Timer(rospy.Duration(1.0), self.open_gripper,
                                      oneshot=True)

    def close_gripper(self):
        self._send_gripper_cmd('c')

    def generate_servo_msg(self, ee_to_base_pos, base_to_world_pos,
                           duration=0.06):
        ''' Convert desired arm and base positions into the format accepted by
            the servo node. '''
        # TODO this should really be a custom message
        servo_data = [0] * 11

        # Populate the servo message.
        servo_data[0] = ee_to_base_pos.x
        servo_data[1] = ee_to_base_pos.y
        servo_data[2] = ee_to_base_pos.z

        servo_data[3] = self.ee_to_base_tf0.rotation.x
        servo_data[4] = self.ee_to_base_tf0.rotation.y
        servo_data[5] = self.ee_to_base_tf0.rotation.z
        servo_data[6] = self.ee_to_base_tf0.rotation.w

        servo_data[7] = base_to_world_pos.x
        servo_data[8] = base_to_world_pos.y

        # Calculate base yaw angle.
        base_quat = np.array([self.base_to_world_tf0.rotation.x,
                              self.base_to_world_tf0.rotation.y,
                              self.base_to_world_tf0.rotation.z,
                              self.base_to_world_tf0.rotation.w])
        base_eul = tf.transformations.euler_from_quaternion(base_quat)
        servo_data[9] = base_eul[2]

        # Final point is the duration of the trajectory.
        servo_data[10] = duration

        return Float64MultiArray(data=servo_data)

    def optimize_redundancy_params(self, ee_tf_act, ee_tf_des):
        ''' Generate the redundancy parameters rho. '''

        # TODO the handling of d may need to get more sophisticated when
        # orientation is considered
        d = self.arm_base_offset
        ee_pos_des = ee_tf_des.translation
        base_pos_act = self.tf_lookup.get('odom', 'base_link').translation

        # Calculate the position of the arm relative to its mounting point on
        # the base.
        # TODO Subtracting d.y interestingly caused the base to move while the
        # EE stayed in the same place, which doesn't make sense to me
        # - does some corresponding change need to be made elsewhere?
        x_arm = ee_pos_des.x - (base_pos_act.x + d.x)
        y_arm = ee_pos_des.y - base_pos_act.y
        z_arm = ee_pos_des.z - d.z

        l_max = 1.4
        l_min = 0.9

        # Max of p1 depends on the current height of the arm.
        p1_max = np.sqrt(l_max**2 - z_arm**2)

        # Bounds on each value of rho.
        lower_bounds = [l_min, -np.pi/4, -np.pi/4]
        upper_bounds = [p1_max, np.pi/4,  np.pi/4]

        # Boring fixed values - this just keeps the configuration constant
        # p1 = 1
        # p2 = p3 = 0
        # return p1, p2, p3, lower_bounds, upper_bounds

        p1 = np.sqrt(x_arm**2 + y_arm**2)
        p3 = np.arcsin(y_arm / p1)

        # Bounds checking.
        # TODO use the fancy bound util method
        if p1 > upper_bounds[0]:
            p1 = upper_bounds[0]
        elif p1 < lower_bounds[0]:
            p1 = lower_bounds[0]

        if p3 > upper_bounds[2]:
            p3 = upper_bounds[2]
        elif p3 < lower_bounds[2]:
            p3 = lower_bounds[2]

        # since we're not considering base orientation at the moment
        p2 = p3

        return p1, p2, p3

    def resolve_redundancy(self, ee_tf_act, ee_tf_des):
        ''' Resolve the redundancy problem between the base and arm. '''
        ee_pos_des = ee_tf_des.translation

        p1, p2, p3 = self.optimize_redundancy_params(ee_tf_act, ee_tf_des)

        ee_to_base_pos = Vector3()
        base_to_world_pos = Vector3()

        d = self.arm_base_offset

        # Calculate the desired end effector position with reference to the
        # to its attachment point.
        ee_to_base_pos.x = p1*np.cos(p2)
        ee_to_base_pos.y = p1*np.sin(p2)
        ee_to_base_pos.z = ee_pos_des.z - d.z

        # Calculate the base position with reference to the world frame.
        # TODO should p3 in the final term correspond to the p2 in the above
        # expressions?
        base_to_world_pos.x = ee_pos_des.x - d.x*np.cos(p3-p2) - p1*np.cos(p3)
        base_to_world_pos.y = ee_pos_des.y - d.y*np.sin(p3-p2) - p1*np.sin(p3)

        return ee_to_base_pos, base_to_world_pos

    def force_control_loop(self):
        ''' Control loop that tracks a trajectory. '''
        # transform from FT frame to world frame
        ft_tf = self.tf_lookup.get_stamped('odom', 'robotiq_force_torque_frame_id')
        f = transform_force(self.force, ft_tf)

        # Force control. Steer toward CONTACT_FORCE if FORCE_THRESHOLD is
        # exceeded.
        comp = np.abs(f) > FORCE_THRESHOLD
        set_point = comp * np.sign(f) * CONTACT_FORCE

        # this makes it such that the force input can only be +-FORCE_THRESHOLD
        f_in = bound_array(f * comp, -MAX_INPUT_FORCE, MAX_INPUT_FORCE)
        p_off = self.force_pid.update(f_in, set_point=set_point)

        # True if force if the force threshold is exceeded along any axis.
        force_detected = comp.sum() > 0

        # Open/close the gripper when we switch.
        if force_detected and not self.force_detected:
            # if gripper is set to open, cancel it; otherwise, close the
            # gripper
            if self.open_timer and self.open_timer.isAlive():
                self.open_timer.shutdown()
            else:
                self.close_gripper()
        elif not force_detected and self.force_detected:
            self.launch_open_timer()

        self.force_detected = force_detected

        # in the odom (world) frame
        ee_tf_des = copy.deepcopy(self.ee_to_world_tf0)
        ee_tf_act = self.get_current_ee_tf()

        # Add position offset from force control to desired EE position in
        # world frame.
        ee_tf_des.translation.x += p_off[0]
        ee_tf_des.translation.y += p_off[1]
        ee_tf_des.translation.z += p_off[2]

        # Perform redundancy resolution.
        ee_to_base_pos, base_to_world_pos = self.resolve_redundancy(ee_tf_act,
                                                                    ee_tf_des)

        # Collect lots of data to log.
        self.logger.force = Vector3(*f)
        self.logger.pos_offset = Vector3(*p_off)
        self.logger.ee_world_des = ee_tf_des.translation
        self.logger.ee_world_act = ee_tf_act.translation
        self.logger.ee_arm_des = ee_to_base_pos
        self.logger.ee_arm_act = ee_tf_act.translation
        self.logger.base_des = base_to_world_pos
        self.logger.base_act = self.tf_lookup.get('odom', 'base_link').translation

        # TODO this gets worse as dt gets smaller, which is problematic for
        # control purposes - is this actually true? probably not a big deal
        # when running onboard
        servo_msg = self.generate_servo_msg(ee_to_base_pos, base_to_world_pos,
                                            duration=self.dt)
        self.pub_goal_servo.publish(servo_msg)

        self.logger.publish()


if __name__ == '__main__':
    rospy.init_node('human_interaction_demo')

    demo = HumanInteractionDemo()
    demo.loop()

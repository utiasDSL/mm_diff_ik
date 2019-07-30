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


class SimpleDemo(object):
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

        # publisher for thing_control node
        self.pub_goal_servo = rospy.Publisher('/servo/command',
                                              Float64MultiArray, queue_size=10)

        # subscriber for FT sensor
        self.sub_FT = rospy.Subscriber('/robotiq_force_torque_wrench_zero',
                                       WrenchStamped, self.ft_callback)

        # zero out FT sensor bias
        rospy.set_param('/FT_sensor_bias_node/set_zero', True)

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
        # control loop
        while not rospy.is_shutdown():
            self.update()
            rospy.sleep(self.dt)

    def get_current_ee_tf(self):
        return self.tf_lookup.get('odom', 'thing_tool')

    def force_control(self):
        ''' Perform force control to generate position offset for parallel
            control. '''
        # transform from force-torque sensor frame to world frame
        ft_tf = self.tf_lookup.get_stamped('odom', 'robotiq_force_torque_frame_id')
        f = transform_force(self.force, ft_tf)

        # Force control. Steer toward CONTACT_FORCE if FORCE_THRESHOLD is
        # exceeded.
        comp = np.abs(f) > FORCE_THRESHOLD
        set_point = comp * np.sign(f) * CONTACT_FORCE

        # this makes it such that the force input can only be +-FORCE_THRESHOLD
        f_in = bound_array(f * comp, -MAX_INPUT_FORCE, MAX_INPUT_FORCE)
        p_off = self.force_pid.update(f_in, set_point=set_point)

        return p_off, f

    def update(self):
        ''' Control loop that tracks a trajectory. '''
        p_off, f = self.force_control()

        # in the odom (world) frame
        ee_tf_des = copy.deepcopy(self.ee_to_world_tf0)
        ee_tf_act = self.get_current_ee_tf()

        # Add position offset from force control to desired EE position in
        # world frame.
        ee_tf_des.translation.x += p_off[0]
        ee_tf_des.translation.y += p_off[1]
        ee_tf_des.translation.z += p_off[2]

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
    rospy.init_node('simple_demo')

    demo = SimpleDemo()
    demo.loop()

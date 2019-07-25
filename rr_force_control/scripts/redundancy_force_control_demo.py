#!/usr/bin/env python2

''' Demo to show redundancy resolution between base and arm with force control.
'''

from __future__ import print_function, division

import rospy
import numpy as np
import sys
import tf.transformations
from PyQt4 import QtGui

from geometry_msgs.msg import Vector3, Vector3Stamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
import tf2_geometry_msgs

from force_pid import ForcePID
from utils import AveragingFilter, TransformLookup
from trajectory import Trajectory, LineTrajectory, SineTrajectory
from logger import Logger


INIT_POSE_DURATION = 10.0  # Seconds
FORCE_THRESHOLD = 5  # Newtons
CONTACT_FORCE = 1  # Desired contact force
NUM_FILTER_PTS = 50  # TODO super high just to make things nice for demo
TIME_STEP = 0.1  # Hz TODO we can push this? may fuck up servo commands then


def transform_force(f, tf):
    ''' Transform force vector f using stamped transform tf. Note that the
        translation component is not used. '''
    # f is an np array of length 3
    v = Vector3Stamped()
    v.vector = Vector3(*f)
    f = tf2_geometry_msgs.do_transform_vector3(v, tf).vector
    return np.array([f.x, f.y, f.z])


class DemoGUI(QtGui.QWidget):
    ''' Simple GUI to control the demo. '''
    def __init__(self, button_info):
        super(DemoGUI, self).__init__()

        # Initialize the GUI window.
        self.setGeometry(500, 150, 300, 300)
        self.setWindowTitle('Thing - Line Draw')

        layout = QtGui.QFormLayout(self)

        # button_info is a 2-tuple: (label [str], handle [func])
        self.buttons = []
        for info in button_info:
            label, handle = info
            button = QtGui.QPushButton(label, self)
            button.clicked[bool].connect(handle)
            self.buttons.append(button)

            layout.addRow(button)


class RedundancyForceControlDemo(object):
    def __init__(self):
        buttons = [('Home Configuration', self.init_cb),
                   ('Forward', self.forward_traj_cb),
                   ('Reverse', self.reverse_traj_cb),
                   ('Left',    self.left_traj_cb),
                   ('Right',   self.right_traj_cb),
                   ('Up',      self.up_traj_cb),
                   ('Down',    self.down_traj_cb),
                   ('Left Sine',  self.sine_traj_left_cb),
                   ('Right Sine', self.sine_traj_right_cb),
                   ('Sequence',   self.seq_cb),
                   ('Stop',    self.stop_cb)]
        self.gui = DemoGUI(buttons)
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
        self.force_pid = ForcePID(set_point=np.zeros(3), reverse_output=True)

        # Trajectory generator.
        self.trajectory = Trajectory(self.ee_to_world_tf0, self.dt, 10)

        # Trajectories to draw DSL
        self.trajs = [(0, 0, 0.1), (0, 0, 0.1), (0, -0.1, 0), (0, 0, -0.1),
                      (0, 0, -0.1), (0, 0.1, 0), (0, -0.1, 0), (0, -0.1, 0), (0, -0.1, 0),
                      (0, 0, 0.1), (0, 0.1, 0), (0, 0, 0.1), (0, -0.1, 0), (0, -0.1, 0),
                      (0, 0, -0.1), (0, 0, -0.1), (0, -0.1, 0)]

        # publisher for thing_control node
        self.pub_goal_servo = rospy.Publisher('/servo/command',
                                              Float64MultiArray, queue_size=10)

        # subscriber for FT sensor
        self.sub_FT = rospy.Subscriber('/robotiq_force_torque_wrench_zero',
                                       WrenchStamped, self.ft_callback)

        # zero out FT sensor bias
        rospy.set_param('/FT_sensor_bias_node/set_zero', True)

        self.pub_timer = None
        self.traj_running = False
        self.contact = False
        self.switch_time = 0
        self.traj_idx = 0
        self.seq_running = False

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

    def init_cb(self, pressed):
        ''' Callback to move to initial configuration and stay there. '''
        if self.pub_timer:
            self.pub_timer.shutdown()
        print('To home!')
        self.traj_running = True
        ee_tf_des = self.ee_to_world_tf0
        ee_tf_act = self.tf_lookup.get('odom', 'thing_tool')

        # Perform redundancy resolution.
        ee_to_base_pos, base_to_world_pos, _, _ = self.resolve_redundancy(ee_tf_act,
                                                                          ee_tf_des)

        servo_msg = self.generate_servo_msg(ee_to_base_pos, base_to_world_pos,
                                            duration=INIT_POSE_DURATION)
        self.pub_goal_servo.publish(servo_msg)

    def seq_cb(self, pressed):
        self.seq_running = True
        lx, ly, lz = self.trajs[0]
        self.launch_line_traj(lx, ly, lz, duration=3.0)

    def stop_cb(self, pressed):
        print('Stopping trajectory.')
        self.traj_running = False
        if self.pub_timer:
            self.pub_timer.shutdown()

    def launch_line_traj(self, lx, ly, lz, duration=3.0):
        if self.pub_timer:
            self.pub_timer.shutdown()

        # If a trajectory is currently running, we start this trajectory from
        # the current desired EE position. Otherwise, we just start from the
        # current actual EE position.
        # if self.traj_running:
        #     tf0 = self.trajectory.get_tf()
        # else:
        tf0 = self.get_current_ee_tf()

        self.trajectory = LineTrajectory(tf0, self.dt, duration, lx, ly, lz)
        self.pub_timer = rospy.Timer(rospy.Duration(self.dt),
                                     self.trajectory_loop)
        self.traj_running = True

    def forward_traj_cb(self, pressed):
        print('Forward.')
        self.launch_line_traj(0.1, 0, 0)

    def reverse_traj_cb(self, pressed):
        print('Reverse.')
        self.launch_line_traj(-0.1, 0, 0)

    def left_traj_cb(self, pressed):
        print('Left.')
        self.launch_line_traj(0, 0.1, 0)

    def right_traj_cb(self, pressed):
        print('Right.')
        self.launch_line_traj(0, -0.1, 0)

    def up_traj_cb(self, pressed):
        print('Up.')
        self.launch_line_traj(0, 0, 0.1)

    def down_traj_cb(self, pressed):
        print('Down.')
        self.launch_line_traj(0, 0, -0.1)

    def sine_traj_left_cb(self, pressed):
        print('Left Sine.')
        if self.pub_timer:
            self.pub_timer.shutdown()

        # If a trajectory is currently running, we start this trajectory from
        # the current desired EE position. Otherwise, we just start from the
        # current actual EE position.
        if self.traj_running:
            tf0 = self.trajectory.get_tf()
        else:
            tf0 = self.get_current_ee_tf()

        self.trajectory = SineTrajectory(tf0, self.dt, duration=20, ly=2, Rz=0.25)
        self.pub_timer = rospy.Timer(rospy.Duration(self.dt),
                                     self.trajectory_loop)
        self.traj_running = True

    def sine_traj_right_cb(self, pressed):
        print('Right Sine.')
        if self.pub_timer:
            self.pub_timer.shutdown()

        # If a trajectory is currently running, we start this trajectory from
        # the current desired EE position. Otherwise, we just start from the
        # current actual EE position.
        if self.traj_running:
            tf0 = self.trajectory.get_tf()
        else:
            tf0 = self.get_current_ee_tf()

        self.trajectory = SineTrajectory(tf0, self.dt, duration=20, ly=-2, Rz=0.25)
        self.pub_timer = rospy.Timer(rospy.Duration(self.dt),
                                     self.trajectory_loop)
        self.traj_running = True

    def ft_callback(self, msg):
        ''' Callback when a force reading is received. '''
        # Pushing on the tip of the end effector results in negative force in
        # the z-direction.
        # msg is a WrenchStamped
        f = msg.wrench.force

        self.force = self.force_filter.filter(np.array([f.x, f.y, f.z]))

        self.logger.force_raw = f
        # self.logger.force = Vector3(*self.force)

    def show(self):
        ''' Show the GUI. '''
        self.gui.show()

    def get_current_ee_tf(self):
        return self.tf_lookup.get('odom', 'thing_tool')  # thing_tool is the way

    def generate_servo_msg(self, ee_to_base_pos, base_to_world_pos,
                           duration=0.06):
        ''' Convert desired arm and base positions into the format accepted by
            the servo node. '''
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

        d = self.arm_base_offset
        ee_pos_des = ee_tf_des.translation
        base_pos_act = self.tf_lookup.get('odom', 'base_link').translation

        # Calculate the position of the arm relative to its mounting point on
        # the base.
        x_arm = ee_pos_des.x - (base_pos_act.x + d.x)
        y_arm = ee_pos_des.y - base_pos_act.y
        z_arm = ee_pos_des.z - d.z

        # Max and min extension of the arm. TODO tune these
        l_max = 1.25
        l_min = 0.75

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
        if p1 > upper_bounds[0]:
            p1 = upper_bounds[0]
        elif p1 < lower_bounds[0]:
            p1 = lower_bounds[0]

        if p3 > upper_bounds[2]:
            p3 = upper_bounds[2]
        elif p3 < lower_bounds[2]:
            p3 = lower_bounds[2]

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
        base_to_world_pos.x = ee_pos_des.x - d.x*np.cos(p3-p2) - p1*np.cos(p3)
        base_to_world_pos.y = ee_pos_des.y - d.y*np.sin(p3-p2) - p1*np.sin(p3)

        return ee_to_base_pos, base_to_world_pos

    def trajectory_loop(self, event):
        ''' Control loop that tracks a trajectory. '''
        done = not self.trajectory.step()
        if done and self.seq_running:
            self.traj_idx += 1
            if self.traj_idx < len(self.trajs):
                lx, ly, lz = self.trajs[self.traj_idx]
                self.launch_line_traj(lx, ly, lz, duration=3.0)

        # transform from FT frame to world frame
        ft_tf = self.tf_lookup.get_stamped('odom', 'robotiq_force_torque_frame_id')
        f = transform_force(self.force, ft_tf)

        # Force control. Steer toward CONTACT_FORCE if FORCE_THRESHOLD is
        # exceeded.
        comp = np.abs(f) > FORCE_THRESHOLD
        set_point = comp * np.sign(f) * CONTACT_FORCE
        pos_offset = self.force_pid.update(f * comp, set_point=set_point)

        # NOTE artificially limit for testing purposes
        pos_offset = np.minimum(np.maximum(pos_offset, -1*np.ones(3)), 1*np.ones(3))

        # in the odom (world) frame
        ee_tf_des = self.trajectory.get_tf()
        ee_tf_act = self.get_current_ee_tf()

        # Add position offset from force control to desired EE position in
        # world frame.
        ee_tf_des.translation.x += pos_offset[0]
        ee_tf_des.translation.y += pos_offset[1]
        ee_tf_des.translation.z += pos_offset[2]

        # print('EEx = {}\npoff = {}\nf = {}'.format(ee_tf_des.translation.x, pos_offset, f))

        # Perform redundancy resolution.
        ee_to_base_pos, base_to_world_pos = self.resolve_redundancy(ee_tf_act,
                                                                    ee_tf_des)

        # Collect lots of data to log.
        self.logger.force = Vector3(*f)
        self.logger.pos_offset = Vector3(*pos_offset)
        self.logger.ee_world_des = ee_tf_des.translation
        self.logger.ee_world_act = ee_tf_act.translation
        self.logger.ee_arm_des = ee_to_base_pos
        self.logger.ee_arm_act = ee_tf_act.translation
        self.logger.base_des = base_to_world_pos
        self.logger.base_act = self.tf_lookup.get('odom', 'base_link').translation

        # TODO this gets worse as dt gets smaller, which is problematic for
        # control purposes
        servo_msg = self.generate_servo_msg(ee_to_base_pos, base_to_world_pos,
                                            duration=self.dt)
        self.pub_goal_servo.publish(servo_msg)

        self.logger.publish()


if __name__ == '__main__':
    rospy.init_node('line_draw')

    try:
        print('Demo launched!')
        gui = QtGui.QApplication(sys.argv)
        demo = RedundancyForceControlDemo()
        demo.show()
        sys.exit(gui.exec_())
    except rospy.ROSInterruptException:
        pass

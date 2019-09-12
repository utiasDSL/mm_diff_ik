#!/usr/bin/env python2
import rospy
import numpy as np
import tf.transformations as tfs

from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from sensor_msgs.msg import JointState
from mm_msgs.msg import ForceControlState

import mm_kinematics.kinematics as kinematics

import mm_force_control.util as util
from mm_force_control.filter import ExponentialSmoother
from mm_force_control.pid import PID
from mm_force_control.bias import FTBiasEstimator


FORCE_THRESHOLD = 5  # Force required for force control to be used
CONTACT_FORCE = 5  # Desired contact force, when contact is detected
MAX_INPUT_FORCE = 10  # Maximum force value that can input to the PID controller

HZ = 20  # Control loop rate (Hz)

N_BIAS = 100  # Number of samples to use for force bias estimation.

# Controller gains
Kp = np.zeros(3)
Kd = np.zeros(3)
Ki = 0.02 * np.ones(3)  # good for HI demo
DECAY = 0

# Transform from EE frame to force torque sensor frame - just a small offset.
e_T_f = tfs.translation_matrix([0.02, 0, 0])


class ForceControlNode(object):
    def __init__(self, bias=np.zeros(3)):
        self.pid = PID(Kp, Ki, Kd, decay=DECAY, desired=np.zeros(3))
        self.smoother = ExponentialSmoother(tau=0.1, x0=np.zeros(3))

        self.bias = bias
        self.force_raw = np.zeros(3)
        self.force_filt = np.zeros(3)
        self.q = np.zeros(9)
        self.time_prev = rospy.Time.now().to_sec()

        # subs/pubs are initialized last since callbacks are multithreaded and
        # can actually be called before other variables in __init__ have been
        # declared
        self.joint_states_sub = rospy.Subscriber('/mm_joint_states',
                                                 JointState,
                                                 self.joint_states_cb)

        self.force_sub = rospy.Subscriber('/robotiq_force_torque_wrench',
                                          WrenchStamped, self.force_cb)

        self.p_off_pub = rospy.Publisher('/force_control/position_offset',
                                         Vector3Stamped, queue_size=10)

        self.state_pub = rospy.Publisher('/force_control/state',
                                         ForceControlState, queue_size=10)

    def force_cb(self, msg):
        f = msg.wrench.force
        self.force_raw = np.array([f.x, f.y, f.z]) - self.bias

        now = rospy.Time.now().to_sec()
        dt = now - self.time_prev
        self.time_prev = now

        # We do filtering in the callback (instead of the control loop) so we
        # can include the maximum number of messages in the filter (for best
        # accuracy)
        self.force_filt = self.smoother.next(self.force_raw, dt)

    def joint_states_cb(self, msg):
        # TODO should handle locking at some point
        self.q = msg.position

    def publish_state(self, stamp, force_raw, force_filt, force_world, p_off):
        msg = ForceControlState()
        msg.header.stamp = stamp

        msg.force_raw.x = force_raw[0]
        msg.force_raw.y = force_raw[1]
        msg.force_raw.z = force_raw[2]

        msg.force_filtered.x = force_filt[0]
        msg.force_filtered.y = force_filt[1]
        msg.force_filtered.z = force_filt[2]

        msg.force_world.x = force_world[0]
        msg.force_world.y = force_world[1]
        msg.force_world.z = force_world[2]

        msg.position_offset.x = p_off[0]
        msg.position_offset.y = p_off[1]
        msg.position_offset.z = p_off[2]

        self.state_pub.publish(msg)

    def publish_position_offset(self, stamp, p_off):
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.vector.x = p_off[0]
        msg.vector.y = p_off[1]
        msg.vector.z = p_off[2]
        self.p_off_pub.publish(msg)

    def loop(self, hz):
        rate = rospy.Rate(hz)
        rospy.loginfo('Force control loop started.')

        while not rospy.is_shutdown():
            # Transform force to the world frame
            w_T_e = kinematics.forward(self.q)
            w_T_f = w_T_e.dot(e_T_f)
            w_R_f = w_T_f[:3,:3]
            force_world = w_R_f.dot(self.force_filt)

            # Force control. Steer toward CONTACT_FORCE if FORCE_THRESHOLD is
            # exceeded.
            comp = np.abs(force_world) > FORCE_THRESHOLD
            desired = comp * np.sign(force_world) * CONTACT_FORCE

            # Bound force input so it can only be between +-FORCE_THRESHOLD
            # force_in = util.bound_array(force_world * comp, -MAX_INPUT_FORCE,
            #                             MAX_INPUT_FORCE)
            force_in = force_world * comp

            # Reverse input so that output offset pushes back against applied
            # force.
            force_in = -force_in

            p_off = self.pid.update(force_in, desired=desired)

            now = rospy.Time.now()
            # self.publish_position_offset(now, np.zeros(3))
            self.publish_position_offset(now, p_off)
            self.publish_state(now, self.force_raw, self.force_filt, force_world, p_off)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('mm_force_control_node')

    # Listen to initial force measurements to estimate sensor bias.
    bias_estimator = FTBiasEstimator(N_BIAS)
    bias_estimator.estimate()
    force_bias = bias_estimator.bias[:3]

    node = ForceControlNode(bias=force_bias)
    node.loop(HZ)

#!/usr/bin/env python2
import rospy
import numpy as np
import tf.transformations as tfs

from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from sensor_msgs.msg import JointState
from mm_msgs.msg import ForceInfo

import mm_kinematics.kinematics as kinematics

import mm_force_control.util as util
from mm_force_control.filter import ExponentialSmoother
from mm_force_control.bias import FTBiasEstimator


CONTACT_THRESHOLD = 5  # Desired contact force, when contact is detected
DESIRED_CONTACT_FORCE = 5

HZ = 20  # Control loop rate (Hz)

N_BIAS = 100  # Number of samples to use for force bias estimation.

# Transform from EE frame to force torque sensor frame - just a small offset.
e_T_f = tfs.translation_matrix([0.02, 0, 0])


class ForceControlNode(object):
    def __init__(self, bias=np.zeros(3)):
        self.smoother = ExponentialSmoother(tau=0.1, x0=np.zeros(3))

        self.bias = bias
        self.force_raw = np.zeros(3)
        self.force_filt = np.zeros(3)
        self.q = np.zeros(9)
        self.time_prev = rospy.Time.now().to_sec()

        self.first_contact = False  # True if contact was ever made
        self.contact = False  # True if EE is currently in contact

        # subs/pubs are initialized last since callbacks are multithreaded and
        # can actually be called before other variables in __init__ have been
        # declared
        self.joint_states_sub = rospy.Subscriber('/mm_joint_states',
                                                 JointState,
                                                 self.joint_states_cb)

        self.force_sub = rospy.Subscriber('/robotiq_force_torque_wrench',
                                          WrenchStamped, self.force_cb)

        # more information about the force processing
        self.info_pub = rospy.Publisher('/force/info', ForceInfo, queue_size=10)

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

    def publish_info(self, stamp, force_world):
        msg = ForceInfo()
        msg.header.stamp = stamp

        msg.force_raw.x = self.force_raw[0]
        msg.force_raw.y = self.force_raw[1]
        msg.force_raw.z = self.force_raw[2]

        msg.force_filtered.x = self.force_filt[0]
        msg.force_filtered.y = self.force_filt[1]
        msg.force_filtered.z = self.force_filt[2]

        msg.force_world.x = force_world[0]
        msg.force_world.y = force_world[1]
        msg.force_world.z = force_world[2]

        msg.first_contact = self.first_contact
        msg.contact = self.contact

        self.info_pub.publish(msg)

    def loop(self, hz):
        rate = rospy.Rate(hz)
        rospy.loginfo('Force processing loop started.')

        while not rospy.is_shutdown():
            # Transform force to the world frame
            w_T_e = kinematics.forward(self.q)
            w_T_f = w_T_e.dot(e_T_f)
            w_R_f = w_T_f[:3, :3]
            force_world = w_R_f.dot(self.force_filt)

            # Reverse input so that output offset pushes back against applied
            # force.
            force_world = -force_world

            # determine if contact is made
            f_norm = np.linalg.norm(force_world)
            if f_norm > CONTACT_THRESHOLD:
                if not self.first_contact:
                    rospy.loginfo('First contact made.')
                self.first_contact = True
                self.contact = True
            else:
                self.contact = False

            now = rospy.Time.now()
            self.publish_info(now, force_world)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('mm_force_node')

    # Listen to initial force measurements to estimate sensor bias.
    bias_estimator = FTBiasEstimator(N_BIAS)
    bias_estimator.estimate()
    force_bias = bias_estimator.bias[:3]

    node = ForceControlNode(bias=force_bias)
    node.loop(HZ)

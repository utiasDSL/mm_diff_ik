#!/usr/bin/env python2
"""Process wrench messages from Robotiq F/T sensor."""
import rospy
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from mm_msgs.msg import WrenchInfo

from mm_kinematics import KinematicModel

from mm_force_torque.filter import ExponentialSmoother
from mm_force_torque.bias import FTBiasEstimator


CONTACT_THRESHOLD = 5  # Force threshold when contact is detected (N)
HZ = 100  # Control loop rate (Hz)
N_BIAS = 100  # Number of samples to use for wrench bias estimation.

# Exponential smoothing time constants
FORCE_FILTER_TAU = 0.05
TORQUE_FILTER_TAU = 0.05


class MMWrenchNode(object):
    def __init__(self, bias=np.zeros(6)):
        self.model = KinematicModel()
        self.force_smoother = ExponentialSmoother(tau=FORCE_FILTER_TAU,
                                                  x0=np.zeros(3))
        self.torque_smoother = ExponentialSmoother(tau=TORQUE_FILTER_TAU,
                                                   x0=np.zeros(3))
        self.bias = bias

        self.force_raw = np.zeros(3)
        self.force_filt = np.zeros(3)
        self.torque_raw = np.zeros(3)
        self.torque_filt = np.zeros(3)

        self.q = np.zeros(9)
        self.time_prev = rospy.Time.now().to_sec()
        self.fd = 0

        self.first_contact = False  # True if contact was ever made
        self.contact = False  # True if EE is currently in contact

        # subs/pubs are initialized last since callbacks are multithreaded and
        # can actually be called before other variables in __init__ have been
        # declared
        self.joint_states_sub = rospy.Subscriber('/mm_joint_states',
                                                 JointState,
                                                 self.joint_states_cb)

        self.wrench_sub = rospy.Subscriber('/robotiq_force_torque_wrench',
                                           WrenchStamped, self.force_cb)

        self.force_des_sub = rospy.Subscriber('/force/desired',
                                              Float64, self.force_des_cb)

        # more information about the force processing
        self.info_pub = rospy.Publisher('/mm_wrench/info', WrenchInfo, queue_size=10)

    def force_cb(self, msg):
        f = msg.wrench.force
        self.force_raw = np.array([f.x, f.y, f.z]) - self.bias[:3]

        tau = msg.wrench.torque
        self.torque_raw = np.array([tau.x, tau.y, tau.z]) - self.bias[3:]

        now = rospy.Time.now().to_sec()
        dt = now - self.time_prev
        self.time_prev = now

        # We do filtering in the callback (instead of the control loop) so we
        # can include the maximum number of messages in the filter (for best
        # accuracy)
        self.force_filt = self.force_smoother.next(self.force_raw, dt)
        self.torque_filt = self.torque_smoother.next(self.torque_raw, dt)

    def force_des_cb(self, msg):
        self.fd = msg.data

    def joint_states_cb(self, msg):
        # TODO should handle locking at some point
        self.q = msg.position

    def publish_info(self, stamp, force_world, torque_world):
        msg = WrenchInfo()
        msg.header.stamp = stamp

        msg.raw.force.x = self.force_raw[0]
        msg.raw.force.y = self.force_raw[1]
        msg.raw.force.z = self.force_raw[2]

        msg.raw.torque.x = self.torque_raw[0]
        msg.raw.torque.y = self.torque_raw[1]
        msg.raw.torque.z = self.torque_raw[2]

        msg.filtered.force.x = self.force_filt[0]
        msg.filtered.force.y = self.force_filt[1]
        msg.filtered.force.z = self.force_filt[2]

        msg.filtered.torque.x = self.torque_filt[0]
        msg.filtered.torque.y = self.torque_filt[1]
        msg.filtered.torque.z = self.torque_filt[2]

        msg.world.force.x = force_world[0]
        msg.world.force.y = force_world[1]
        msg.world.force.z = force_world[2]

        msg.world.torque.x = torque_world[0]
        msg.world.torque.y = torque_world[1]
        msg.world.torque.z = torque_world[2]

        msg.force_desired = self.fd

        msg.first_contact = self.first_contact
        msg.contact = self.contact

        self.info_pub.publish(msg)

    def loop(self, hz):
        rate = rospy.Rate(hz)
        rospy.loginfo('Force processing loop started.')

        while not rospy.is_shutdown():
            # Transform force to the world frame
            w_T_ft = self.model.calc_T_w_ft(self.q)
            w_R_ft = w_T_ft[:3, :3]
            force_world = w_R_ft.dot(self.force_filt)
            torque_world = w_R_ft.dot(self.torque_filt)

            # Reverse input so that output offset pushes back against applied
            # force.
            force_world = -force_world
            torque_world = -torque_world

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
            self.publish_info(now, force_world, torque_world)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('mm_wrench_node')

    # Listen to initial force measurements to estimate sensor bias.
    bias_estimator = FTBiasEstimator(N_BIAS)
    bias_estimator.estimate()

    node = MMWrenchNode(bias=bias_estimator.bias)
    node.loop(HZ)

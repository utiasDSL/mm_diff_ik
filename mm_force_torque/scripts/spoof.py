#!/usr/bin/env python2
# TODO this script must be updated: or, instead, write a robotiq sensor
# simulator
import rospy
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from mm_msgs.msg import ForceInfo

import mm_kinematics.kinematics as kinematics


CONTACT_THRESHOLD = 5  # Threshold when contact is detected

HZ = 100  # Control loop rate (Hz)

# position:
# x: 1.48962656096
# y: -0.153941
# z: 0.692672139213

P0 = np.array([1.5, 0, 0])
N0 = np.array([-1, -1, 0])
N0 = N0 / np.linalg.norm(N0)
Kf = 1e3


class SpoofForceNode(object):
    def __init__(self):
        self.force_raw = np.zeros(3)
        self.force_filt = np.zeros(3)
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

        self.force_des_sub = rospy.Subscriber('/force/desired',
                                              Float64, self.force_des_cb)

        # more information about the force processing
        self.info_pub = rospy.Publisher('/force/info', ForceInfo, queue_size=10)

    def force_des_cb(self, msg):
        self.fd = msg.data

    def joint_states_cb(self, msg):
        self.q = msg.position

    def publish_info(self, stamp, force_world):
        msg = ForceInfo()
        msg.header.stamp = stamp

        # msg.force_raw.x = self.force_raw[0]
        # msg.force_raw.y = self.force_raw[1]
        # msg.force_raw.z = self.force_raw[2]

        # msg.force_filtered.x = self.force_filt[0]
        # msg.force_filtered.y = self.force_filt[1]
        # msg.force_filtered.z = self.force_filt[2]

        msg.force_world.x = force_world[0]
        msg.force_world.y = force_world[1]
        msg.force_world.z = force_world[2]

        msg.force_desired = self.fd

        msg.first_contact = self.first_contact
        msg.contact = self.contact

        self.info_pub.publish(msg)

    def loop(self, hz):
        rate = rospy.Rate(hz)
        rospy.loginfo('Force processing loop started.')

        while not rospy.is_shutdown():
            w_T_tool = kinematics.calc_w_T_tool(self.q)
            pe = w_T_tool[:3, 3]
            d = (pe - P0).T.dot(N0)

            # calculate normal vector of EE in world frame
            Re = w_T_tool[:3, :3]
            z = np.array([0, 0, 1])
            ne = Re.dot(z)

            if d < 0:
                f = Kf * d * N0
            else:
                f = np.zeros(3)

            print('d = {}'.format(d))
            print('f = {}'.format(f))
            # print('ne = {}'.format(ne))

            # determine if contact is made
            f_norm = np.linalg.norm(f)
            if f_norm > CONTACT_THRESHOLD:
                if not self.first_contact:
                    rospy.loginfo('First contact made.')
                self.first_contact = True
                self.contact = True
            else:
                self.contact = False

            now = rospy.Time.now()
            self.publish_info(now, f)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('mm_force_node')
    node = SpoofForceNode()
    node.loop(HZ)

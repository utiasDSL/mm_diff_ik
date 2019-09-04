#!/usr/bin/env python2

from __future__ import print_function

import rospy
import numpy as np

from sensor_msgs.msg import JointState


class JointInitializer(object):
    ''' Initialize joints by waiting for '''
    def __init__(self):
        self.joint_state_sub = rospy.Subscriber('/mm_joint_states', JointState,
                                                self.joint_state_cb)
        self.initialized = False

    def joint_state_cb(self, msg):
        self.q0 = np.array(msg.position)
        self.dq0 = np.array(msg.velocity)

        # Once we have a message, we don't need to listen any more
        self.joint_state_sub.unregister()
        self.initialized = True

    def state(self):
        return self.q0, self.dq0

    @staticmethod
    def wait_for_msg(dt):
        j = JointInitializer()
        while not rospy.is_shutdown() and not j.initialized:
            rospy.sleep(dt)
        return j.state()

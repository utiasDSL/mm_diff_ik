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


class LineTrajectory(object):
    ''' Straight line in x-direction. '''
    def __init__(self, p0):
        self.p0 = p0
        self.t0 = rospy.get_time()

    def sample(self, t):
        # 1cm/second
        x = self.p0[0] + 0.01 * (t - self.t0)
        y = self.p0[1]
        z = self.p0[2]

        dx = 0.01
        dy = dz = 0

        return np.array([x, y, z]), np.array([dx, dy, dz])


class SineTrajectory(object):
    def __init__(self, p0):
        self.p0 = p0
        self.t0 = rospy.get_time()

    def sample(self, t):
        v = 0.05
        w = 0.1

        x = self.p0[0] + v * (t - self.t0)
        y = self.p0[1] + np.sin(w*(t - self.t0))
        z = self.p0[2]

        dx = v
        dy = w * np.cos(w*(t-self.t0))
        dz = 0

        return np.array([x, y, z]), np.array([dx, dy, dz])


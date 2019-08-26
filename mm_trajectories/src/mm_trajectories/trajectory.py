#!/usr/bin/env python2

from __future__ import print_function

import rospy
import numpy as np
import tf.transformations as tfs

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


class StationaryTrajectory(object):
    ''' No movement from initial pose. '''
    def __init__(self, p0, quat0):
        self.p0 = p0
        self.quat0 = quat0
        self.t0 = rospy.get_time()

    def sample_linear(self, t):
        return self.p0, np.zeros(3)

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)


class LineTrajectory(object):
    ''' Straight line in x-direction. '''
    def __init__(self, p0, quat0):
        self.p0 = p0
        self.quat0 = quat0
        self.t0 = rospy.get_time()

    def sample_linear(self, t):
        v = 0.05

        x = self.p0[0] + v * (t - self.t0)
        y = self.p0[1]
        z = self.p0[2]

        dx = v
        dy = dz = 0

        return np.array([x, y, z]), np.array([dx, dy, dz])

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)


class SineTrajectory(object):
    ''' Move forward while the EE traces a sine wave in the x-y plane. '''
    def __init__(self, p0, quat0):
        self.p0 = p0
        self.quat0 = quat0
        self.t0 = rospy.get_time()

    def sample_linear(self, t):
        v = 0.05
        w = 0.1

        x = self.p0[0] + v * (t - self.t0)
        y = self.p0[1] + np.sin(w*(t - self.t0))
        z = self.p0[2]

        dx = v
        dy = w * np.cos(w*(t-self.t0))
        dz = 0

        return np.array([x, y, z]), np.array([dx, dy, dz])

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)


class RotationalTrajectory(object):
    ''' Rotate the EE about the z-axis with no linear movement. '''
    def __init__(self, p0, quat0):
        self.p0 = p0
        self.quat0 = quat0
        self.t0 = rospy.get_time()

    def sample_linear(self, t):
        # Positions do not change
        return self.p0, np.zeros(3)

    def sample_rotation(self, t):
        # small rotation about the z-axis
        w = 0.05
        theta = w * t
        axis = np.array([0, 0, 1])

        dq = tfs.quaternion_about_axis(theta, axis)
        quat = tfs.quaternion_multiply(dq, self.quat0)

        return quat, w * axis


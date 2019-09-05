#!/usr/bin/env python2

from __future__ import print_function, division

import rospy
import numpy as np
import tf.transformations as tfs

from mm_msgs.msg import PoseTrajectory

import mm_kinematics.kinematics as kinematics
import mm_msgs.conversions as conversions
from mm_trajectories.util import JointInitializer


def launch(trajectory, duration, dt=0.1):
    traj_pub = rospy.Publisher('/trajectory/poses', PoseTrajectory,
                               queue_size=10)

    # Need to wait a second between setting up the publisher and actually using
    # it to publish a message.
    rospy.sleep(1.0)

    # wait until current joint state is received
    q0, dq0 = JointInitializer.wait_for_msg(dt)

    # calculate initial EE position and velocity
    T0 = kinematics.forward(q0)
    p0 = tfs.translation_from_matrix(T0)
    quat0 = tfs.quaternion_from_matrix(T0)

    print('Launched {} with duration of {} seconds.'.format(
        type(trajectory).__name__, duration))

    traj = trajectory(p0, quat0, duration)
    waypoints = []
    t = 0
    tf = duration

    N = int(tf / dt) + 1
    for _ in xrange(N):
        p, v = traj.sample_linear(t)
        q, w = traj.sample_rotation(t)

        waypoint = conversions.waypoint_msg(t, p, v, q, w)

        waypoints.append(waypoint)

        t += dt

    msg = PoseTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.points = waypoints
    msg.dt = rospy.Duration(dt)

    traj_pub.publish(msg)


# class StationaryTrajectory(object):
#     ''' No movement from initial pose. '''
#     def __init__(self, p0, quat0):
#         self.p0 = p0
#         self.quat0 = quat0
#
#     def sample_linear(self, t):
#         return self.p0, np.zeros(3)
#
#     def sample_rotation(self, t):
#         return self.quat0, np.zeros(3)


class LineTrajectory(object):
    ''' Straight line in x-direction. '''
    def __init__(self, p0, quat0, duration):
        self.p0 = p0
        self.quat0 = quat0
        self.t0 = 0

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


class SineXYTrajectory(object):
    ''' Move forward while the EE traces a sine wave in the x-y plane. '''
    def __init__(self, p0, quat0, duration):
        self.p0 = p0
        self.quat0 = quat0
        self.t0 = 0

    def sample_linear(self, t):
        v = 0.05
        w = 0.2

        x = self.p0[0] + v * (t - self.t0)
        y = self.p0[1] + np.sin(w*(t - self.t0))
        z = self.p0[2]

        dx = v
        dy = w * np.cos(w*(t-self.t0))
        dz = 0

        return np.array([x, y, z]), np.array([dx, dy, dz])

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)


class SineYZTrajectory(object):
    ''' Move sideways while the EE traces a sine wave in the y-z plane. '''
    def __init__(self, p0, quat0, duration):
        self.p0 = p0
        self.quat0 = quat0

    def sample_linear(self, t):
        v = 0.05
        w = 0.2

        x = self.p0[0]
        y = self.p0[1] + v * t
        z = self.p0[2] + np.sin(w * t)

        dx = 0
        dy = v
        dz = w * np.cos(w * t)

        return np.array([x, y, z]), np.array([dx, dy, dz])

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)


class RotationalTrajectory(object):
    ''' Rotate the EE about the z-axis with no linear movement. '''
    def __init__(self, p0, quat0, duration):
        self.p0 = p0
        self.quat0 = quat0

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


# TODO need to deal with API difference in launch
# could just shittily define R here and pass duration in to all of them
class CircleTrajectory(object):
    def __init__(self, p0, quat0, duration):
        self.p0 = p0
        self.quat0 = quat0
        self.w = 2 * np.pi / duration

    def sample_linear(self, t):
        R = 0.3

        x = self.p0[0]
        y = self.p0[1] + R * np.cos(self.w * t) - R
        z = self.p0[2] + R * np.sin(self.w * t)

        dx = 0
        dy = -self.w * R * np.sin(self.w * t)
        dz =  self.w * R * np.cos(self.w * t)

        return np.array([x, y, z]), np.array([dx, dy, dz])

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)


class SquareTrajectory(object):
    def __init__(self, p0, quat0, duration):
        self.p0 = p0
        self.quat0 = quat0
        self.duration = duration

    def sample_linear(self, t):
        R = 1

        x = self.p0[0]
        y = self.p0[1]
        z = self.p0[2]

        dx = dy = dz = 0

        # Total distance travelled is 8 * R; d is the current distance
        # travelled
        v = 8 * R / self.duration
        d = v * t

        if t < self.duration / 8.0:
            y -= d
            dy = -v
        elif t < self.duration * 3.0 / 8.0:
            x -= d - R
            y -= R
            dx = -v
        elif t < self.duration * 5.0 / 8.0:
            x -= 2*R
            y += d - 4*R
            dy = v
        elif t < self.duration * 7.0 / 8.0:
            x += d - 7*R
            y += R
            dx = v
        else:
            y += 8*R - d
            dy = -v

        return np.array([x, y, z]), np.array([dx, dy, dz])

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)

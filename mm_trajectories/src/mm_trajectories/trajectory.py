#!/usr/bin/env python2

from __future__ import print_function, division

import rospy
import numpy as np
import tf.transformations as tfs

from mm_msgs.msg import PoseTrajectory

import mm_kinematics.kinematics as kinematics
import mm_msgs.conversions as conversions
from mm_trajectories.util import JointInitializer


def create_waypoints(traj, duration, dt):
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

    return waypoints


def launch(trajectory, duration, dt=0.1):
    traj_pub = rospy.Publisher('/trajectory/poses', PoseTrajectory,
                               queue_size=10)

    # Need to wait a second between setting up the publisher and actually using
    # it to publish a message.
    rospy.sleep(1.0)

    # wait until current joint state is received
    q0, dq0 = JointInitializer.wait_for_msg(dt)

    # calculate initial tool position and velocity
    T0 = kinematics.calc_w_T_tool(q0)
    p0 = tfs.translation_from_matrix(T0)
    quat0 = tfs.quaternion_from_matrix(T0)

    traj = trajectory(p0, quat0, duration)
    waypoints = create_waypoints(traj, duration, dt)

    msg = PoseTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.points = waypoints
    msg.dt = rospy.Duration(dt)

    traj_pub.publish(msg)

    print('Launched {} with duration of {} seconds.'.format(
        traj.name, duration))


def trapezoidal_velocity(v_max, t, t_acc, duration):
    ''' Trapezoidal velocity profile '''
    if t < t_acc:
        acc = v_max / t_acc
        vel = t * acc
    elif t >= t_acc and t < duration - t_acc:
        acc = 0
        vel = v_max
    else:
        acc = -v_max / t_acc
        vel = (duration - t) * v_max / t_acc
    return vel, acc


class LineTrajectory(object):
    ''' Straight line in x-direction. '''
    def __init__(self, p0, quat0, duration, dt=0.1):
        self.name = 'Line'
        self.p0 = p0
        self.quat0 = quat0
        self.duration = duration
        self.t_acc = 0.1 * self.duration
        self.dt = dt

    def sample_linear(self, t):
        # v_max = 0.5  # for speed
        v_max = 0.1
        # v, a = trapezoidal_velocity(v_max, t, self.t_acc, self.duration)

        v = v_max

        # p = self.p0 + np.array([v * t, 0, 0])
        self.p0[0] += v * self.dt

        dx = v
        dy = dz = 0

        # return p, np.array([dx, dy, dz])
        return self.p0, np.array([dx, dy, dz])

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)


class SineXYTrajectory(object):
    ''' Move forward while the EE traces a sine wave in the x-y plane. '''
    def __init__(self, p0, quat0, duration, dt=0.1):
        self.name = 'Sine'
        self.p0 = p0
        self.quat0 = quat0
        self.duration = duration
        self.t_acc = 0.1 * duration
        self.dt = dt

    def sample_linear(self, t):
        # v = 0.25 and w = 1.0 are as aggressive as I want to go, but they are
        # quite impressive
        v_max = 0.1  # linear velocity - default 0.1
        A_max = 1.0  # sine amplitude - default 1
        # w = 0.25  # sine frequency
        w = 2 * np.pi / self.duration

        # v = trapezoidal_velocity(v_max, t, self.t_acc, self.duration)
        # A = trapezoidal_velocity(A_max, t, self.t_acc, self.duration)
        v = v_max
        A = A_max

        p = self.p0 + np.array([v * t, A * np.sin(w * t), 0])
        # self.p[0] += v * self.dt
        # self.p[1] = A * np.sin(w * t)

        # x = self.p0[0] + v * t
        # y = self.p0[1] + A * np.sin(w * t)
        # z = self.p0[2]

        dx = v
        dy = w * A * np.cos(w * t)
        dz = 0

        return p, np.array([dx, dy, dz])

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)


class RotationalTrajectory(object):
    ''' Rotate the EE about the z-axis with no linear movement. '''
    def __init__(self, p0, quat0, duration):
        self.name = 'Rotational'
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


class CircleTrajectory(object):
    def __init__(self, p0, quat0, duration):
        self.name = 'Figure 8'
        self.p0 = p0
        self.quat0 = quat0
        self.duration = duration
        self.w = 2 * np.pi / (duration / 2.0)

    def sample_linear(self, t):
        R = 0.2

        x = self.p0[0]
        dx = 0

        if t < self.duration / 2.0:
            y = self.p0[1] + R * np.cos(self.w * t) - R
            z = self.p0[2] + R * np.sin(self.w * t)

            dy = -self.w * R * np.sin(self.w * t)
            dz =  self.w * R * np.cos(self.w * t)
        else:
            # y is negated, z stays the same
            y = self.p0[1] - R * np.cos(self.w * t) + R
            z = self.p0[2] + R * np.sin(self.w * t)

            dy = self.w * R * np.sin(self.w * t)
            dz = self.w * R * np.cos(self.w * t)

        return np.array([x, y, z]), np.array([dx, dy, dz])

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)


class SpiralTrajectory(object):
    def __init__(self, p0, quat0, duration, dt=0.1):
        self.name = 'Spiral'
        self.p0 = p0
        self.quat0 = quat0
        self.duration = duration

    def sample_linear(self, t):
        v = 0.1
        w = 0.5
        r = 0.01
        R = r * t

        x = self.p0[0] + v * t
        y = self.p0[1] + R * np.cos(w * t) - R
        z = self.p0[2] + R * np.sin(w * t)

        dx = v
        dy = r*(np.cos(w*t) - w*t*np.sin(w*t) - 1)
        dz = r*(np.sin(w*t) + w*t*np.cos(w*t))

        return np.array([x, y, z]), np.array([dx, dy, dz])

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)


class SquareTrajectory(object):
    def __init__(self, p0, quat0, duration):
        self.name = 'Square'
        self.p0 = p0
        self.quat0 = quat0
        self.duration = duration

    def sample_linear(self, t):
        R = 0.5

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
            x += d - R
            y -= R
            dx = v
        elif t < self.duration * 5.0 / 8.0:
            x += 2*R
            y += d - 4*R
            dy = v
        elif t < self.duration * 7.0 / 8.0:
            x -= d - 7*R
            y += R
            dx = -v
        else:
            y += 8*R - d
            dy = -v

        return np.array([x, y, z]), np.array([dx, dy, dz])

    def sample_rotation(self, t):
        return self.quat0, np.zeros(3)

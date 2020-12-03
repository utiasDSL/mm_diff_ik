#!/usr/bin/env python2

from __future__ import print_function, division

import numpy as np
import tf.transformations as tfs


class PointToPoint:
    ''' Point-to-point trajectory. '''
    def __init__(self, p0, p1, quat, timescaling, duration):
        self.p0 = p0
        self.p1 = p1
        self.quat = quat
        self.timescaling = timescaling
        self.duration = duration

    def sample_linear(self, t):
        s, ds, dds = self.timescaling.eval(t)
        p = self.p0 + (s * (self.p1 - self.p0)[:, None]).T
        v = (ds * (self.p1 - self.p0)[:, None]).T
        a = (dds * (self.p1 - self.p0)[:, None]).T
        return p, v, a

    def sample_rotation(self, t):
        n = t.shape[0]
        return np.tile(self.quat, (n, 1)), np.zeros((n, 3)), np.zeros((n, 3))


class Circle:
    ''' Circular trajectory. '''
    def __init__(self, p0, r, quat, timescaling, duration):
        ''' p0 is the starting point; center is p0 + [r, 0] '''
        self.r = r
        self.quat = quat
        self.pc = p0 + np.array([0, -r, 0])  # start midway up left side of circle
        self.timescaling = timescaling
        self.duration = duration

    def sample_linear(self, t):
        s, ds, dds = self.timescaling.eval(t)

        cs = np.cos(2*np.pi*s)
        ss = np.sin(2*np.pi*s)
        p = self.pc + self.r * np.array([np.zeros_like(t), cs, ss]).T

        dpds = 2*np.pi*self.r * np.array([np.zeros_like(t), -ss, cs])
        v = dpds * ds

        dpds2 = 4*np.pi**2*self.r * np.array([np.zeros_like(t), -cs, -ss])
        a = dpds * dds + dpds2 * ds**2

        # ugly transpose tricks to make dimensions work out
        return p, v.T, a.T

    def sample_rotation(self, t):
        n = t.shape[0]
        return np.tile(self.quat, (n, 1)), np.zeros((n, 3)), np.zeros((n, 3))


class SineXY:
    ''' Sinusoidal trajectory: linear in x, sinusoidal in y, constant in z. '''
    def __init__(self, p0, quat0, lx, amp, freq, timescaling, duration):
        self.p0 = p0
        self.quat = quat0
        self.lx = lx
        self.A = amp

        # multiply by 2*pi so that sin(s) = sin(2*pi) at s = 1
        self.w = freq * 2 * np.pi

        self.timescaling = timescaling
        self.duration = duration

    def sample_linear(self, t):
        s, ds, dds = self.timescaling.eval(t)

        # x is linear in s
        x = self.p0[0] + self.lx * s
        dx = self.lx * ds
        ddx = self.lx * dds

        # y = A*sin(w*s)
        y = self.p0[1] + self.A*np.sin(self.w*s)
        dyds = self.A*self.w*np.cos(self.w*s)
        dyds2 = -self.w**2 * y
        dy = dyds * ds
        ddy = dyds * dds + dyds2 * ds**2

        # z = const
        z = self.p0[2] * np.ones_like(t)
        dz = ddz = np.zeros_like(t)

        p = np.vstack((x, y, z)).T
        v = np.vstack((dx, dy, dz)).T
        a = np.vstack((ddx, ddy, ddz)).T

        return p, v, a

    def sample_rotation(self, t):
        n = t.shape[0]
        return np.tile(self.quat, (n, 1)), np.zeros((n, 3)), np.zeros((n, 3))


# == Old trajectories == #

class LineTrajectory(object):
    ''' Straight line in x-direction. '''
    def __init__(self, p0, quat0, duration, dt=0.1):
        self.name = 'Line'
        self.p0 = p0
        self.quat0 = quat0
        self.duration = duration
        self.dt = dt

    def sample_linear(self, t):
        v = np.array([0.1, 0, 0])
        p = self.p0 + t * v
        return p, v

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
    ''' Rotate pi/2 about z then pi/2 about y '''
    def __init__(self, p0, quat0, duration):
        self.name = 'Rotational'
        self.p0 = p0
        self.quat0 = quat0
        self.duration = duration

    def sample_linear(self, t):
        # Positions do not change
        return self.p0, np.zeros(3)

    def sample_rotation(self, t):
        # small rotation about the z-axis
        w = np.pi / self.duration
        az = np.array([0, 0, 1])
        ay = np.array([-1, 0, 0])

        if t < self.duration / 2.0:
            rz = w * t
            ry = 0.0
            axis = az
        else:
            rz = w * self.duration / 2.0
            ry = w * (t - self.duration / 2.0)
            axis = ay

        qy = tfs.quaternion_about_axis(ry, ay)
        qz = tfs.quaternion_about_axis(rz, az)
        dq = tfs.quaternion_multiply(qy, qz)
        quat = tfs.quaternion_multiply(dq, self.quat0)

        return quat, w * axis


class CircleTrajectory(object):
    def __init__(self, p0, quat0, duration):
        self.name = 'Figure 8'
        self.p0 = p0
        self.quat0 = quat0
        self.duration = duration
        # self.w = 2 * np.pi / (duration / 2.0)
        self.w = 2 * np.pi / duration

    def sample_linear(self, t):
        R = 0.2

        x = self.p0[0]
        dx = 0

        # if t < self.duration / 2.0:
        y = self.p0[1] + R * np.cos(self.w * t) - R
        z = self.p0[2] + R * np.sin(self.w * t)

        dy = -self.w * R * np.sin(self.w * t)
        dz =  self.w * R * np.cos(self.w * t)
        # else:
        #     # y is negated, z stays the same
        #     y = self.p0[1] - R * np.cos(self.w * t) + R
        #     z = self.p0[2] + R * np.sin(self.w * t)
        #
        #     dy = self.w * R * np.sin(self.w * t)
        #     dz = self.w * R * np.cos(self.w * t)

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

#!/usr/bin/env python2

from __future__ import print_function, division

import numpy as np
import tf.transformations as tfs


class PointToPoint(object):
    """Point-to-point trajectory."""
    def __init__(self, p0, p1, quat, timescaling):
        self.p0 = p0
        self.p1 = p1
        self.quat = quat
        self.timescaling = timescaling

    def sample_linear(self, t):
        s, ds, dds = self.timescaling.eval(t)
        p = self.p0 + (s * (self.p1 - self.p0)[:, None]).T
        v = (ds * (self.p1 - self.p0)[:, None]).T
        a = (dds * (self.p1 - self.p0)[:, None]).T
        return p, v, a

    def sample_rotation(self, t):
        n = t.shape[0]
        return np.tile(self.quat, (n, 1)), np.zeros((n, 3)), np.zeros((n, 3))


class Circle(object):
    """Circular trajectory."""
    def __init__(self, p0, quat0, r, timescaling):
        # p0 is the starting point; center is p0 + [0, -r, 0]
        self.r = r
        self.quat0 = quat0
        self.pc = p0 + np.array([0, -r, 0])  # start midway up left side of circle
        self.timescaling = timescaling

    def sample_linear(self, t):
        s, ds, dds = self.timescaling.eval(t)

        cs = np.cos(2*np.pi*s)
        ss = np.sin(2*np.pi*s)

        p = self.pc + self.r * np.stack((np.zeros_like(t), cs, ss), axis=1)
        dpds = 2*np.pi*self.r * np.stack((np.zeros_like(t), -ss, cs), axis=1)
        dpds2 = 4*np.pi**2*self.r * np.stack((np.zeros_like(t), -cs, -ss), axis=1)

        v = dpds * ds[:, None]
        a = dpds * dds[:, None] + dpds2 * ds[:, None]**2

        return p, v, a

    def sample_rotation(self, t):
        n = t.shape[0]
        return np.tile(self.quat0, (n, 1)), np.zeros((n, 3)), np.zeros((n, 3))


class Figure8(object):
    """Figure 8 trajectory."""
    def __init__(self, p0, quat0, r, timescaling):
        self.r = r
        self.quat0 = quat0
        self.pc = p0 + np.array([0, -r, 0])
        self.timescaling = timescaling

    def sample_linear(self, t):
        s, ds, dds = self.timescaling.eval(t)

        # first half: circle to the right
        cs1 = np.cos(4*np.pi*s[s <= 0.5])
        ss1 = np.sin(4*np.pi*s[s <= 0.5])

        # second half: circle to the left
        cs2 = np.cos(4*np.pi*s[s > 0.5])
        ss2 = np.sin(4*np.pi*s[s > 0.5])

        # put together
        y = np.concatenate((cs1, 2 - cs2))
        z = np.concatenate((ss1, ss2))

        # first and second time derivatives
        dy_ds = np.concatenate((-ss1, ss2))
        dz_ds = np.concatenate((cs1, cs2))

        dy_ds2 = np.concatenate((-cs1, cs2))
        dz_ds2 = np.concatenate((-ss1, -ss2))

        # these are all of shape (n, 3)
        p = self.pc + self.r * np.stack((np.zeros_like(t), y, z), axis=1)
        dpds = 2*np.pi*self.r * np.stack((np.zeros_like(t), dy_ds, dz_ds), axis=1)
        dpds2 = 4*np.pi**2*self.r * np.stack((np.zeros_like(t), dy_ds2, dz_ds2), axis=1)

        v = dpds * ds[:, None]
        a = dpds * dds[:, None] + dpds2 * ds[:, None]**2

        return p, v, a

    def sample_rotation(self, t):
        n = t.shape[0]
        return np.tile(self.quat0, (n, 1)), np.zeros((n, 3)), np.zeros((n, 3))


class SineXY(object):
    """Sinusoidal trajectory: linear in x, sinusoidal in y, constant in z."""
    def __init__(self, p0, quat0, lx, amp, freq, timescaling):
        self.p0 = p0
        self.quat = quat0
        self.lx = lx
        self.A = amp

        # multiply by 2*pi so that sin(s) = sin(2*pi) at s = 1
        self.w = freq * 2 * np.pi

        self.timescaling = timescaling

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


class Spiral(object):
    """Spiral trajectory."""
    def __init__(self, p0, quat0, r, freq, lx, timescaling):
        # this is most similar to the sine trajectory
        self.p0 = p0
        self.quat0 = quat0
        self.r = r
        self.w = freq * 2 * np.pi
        self.lx = lx

        self.timescaling = timescaling

    def sample_linear(self, t):
        s, dsdt, dsdt2 = self.timescaling.eval(t)

        R = self.r * s

        # x is linear in s
        x = self.p0[0] + self.lx * s
        dxdt = self.lx * dsdt
        dxdt2 = self.lx * dsdt2

        y = self.p0[1] + R * np.cos(self.w * s) - R
        dyds = -R * self.w * np.sin(self.w * s)
        dyds2 = -R * self.w**2 * np.cos(self.w * s)
        dydt = dyds * dsdt
        dydt2 = dyds * dsdt2 + dyds2 * dsdt**2

        z = self.p0[2] + R * np.sin(self.w * s)
        dzds = R * self.w * np.cos(self.w * s)
        dzds2 = -R * self.w**2 * np.sin(self.w * s)
        dzdt = dzds * dsdt
        dzdt2 = dzds * dsdt2 + dzds2 * dsdt**2

        p = np.stack((x, y, z), axis=1)
        v = np.stack((dxdt, dydt, dzdt), axis=1)
        a = np.stack((dxdt2, dydt2, dzdt2), axis=1)

        return p, v, a

    def sample_rotation(self, t):
        n = t.shape[0]
        return np.tile(self.quat0, (n, 1)), np.zeros((n, 3)), np.zeros((n, 3))


class Rotational(object):
    """Rotate by `angle` about `axis`."""
    def __init__(self, p0, quat0, axis, angle, timescaling):
        self.p0 = p0
        self.quat0 = quat0
        self.axis = axis
        self.angle = angle  # from current orientation
        self.timescaling = timescaling

    def sample_linear(self, t):
        n = t.shape[0]
        return np.tile(self.p0, (n, 1)), np.zeros((n, 3)), np.zeros((n, 3))

    def sample_rotation(self, t):
        s, dsdt, dsdt2 = self.timescaling.eval(t)

        a = self.angle * s
        dadt = self.angle * dsdt
        dadt2 = self.angle * dsdt2

        quats = np.zeros((t.shape[0], 4))
        for i in xrange(t.shape[0]):
            qa = tfs.quaternion_about_axis(a[i], self.axis)
            quats[i, :] = tfs.quaternion_multiply(qa, self.quat0)

        vel = dadt[:, None] * self.axis
        acc = dadt2[:, None] * self.axis

        return quats, vel, acc


# class RotationalTrajectory(object):
#     ''' Rotate pi/2 about z then pi/2 about y '''
#     def __init__(self, p0, quat0, duration):
#         self.name = 'Rotational'
#         self.p0 = p0
#         self.quat0 = quat0
#         self.duration = duration
#
#     def sample_linear(self, t):
#         # Positions do not change
#         return self.p0, np.zeros(3)
#
#     def sample_rotation(self, t):
#         # small rotation about the z-axis
#         w = np.pi / self.duration
#         az = np.array([0, 0, 1])
#         ay = np.array([-1, 0, 0])
#
#         if t < self.duration / 2.0:
#             rz = w * t
#             ry = 0.0
#             axis = az
#         else:
#             rz = w * self.duration / 2.0
#             ry = w * (t - self.duration / 2.0)
#             axis = ay
#
#         qy = tfs.quaternion_about_axis(ry, ay)
#         qz = tfs.quaternion_about_axis(rz, az)
#         dq = tfs.quaternion_multiply(qy, qz)
#         quat = tfs.quaternion_multiply(dq, self.quat0)
#
#         return quat, w * axis


# class SquareTrajectory(object):
#     def __init__(self, p0, quat0, duration):
#         self.name = 'Square'
#         self.p0 = p0
#         self.quat0 = quat0
#         self.duration = duration
#
#     def sample_linear(self, t):
#         R = 0.5
#
#         x = self.p0[0]
#         y = self.p0[1]
#         z = self.p0[2]
#
#         dx = dy = dz = 0
#
#         # Total distance travelled is 8 * R; d is the current distance
#         # travelled
#         v = 8 * R / self.duration
#         d = v * t
#
#         if t < self.duration / 8.0:
#             y -= d
#             dy = -v
#         elif t < self.duration * 3.0 / 8.0:
#             x += d - R
#             y -= R
#             dx = v
#         elif t < self.duration * 5.0 / 8.0:
#             x += 2*R
#             y += d - 4*R
#             dy = v
#         elif t < self.duration * 7.0 / 8.0:
#             x -= d - 7*R
#             y += R
#             dx = -v
#         else:
#             y += 8*R - d
#             dy = -v
#
#         return np.array([x, y, z]), np.array([dx, dy, dz])
#
#     def sample_rotation(self, t):
#         return self.quat0, np.zeros(3)

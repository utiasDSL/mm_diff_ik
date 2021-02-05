from __future__ import division
import numpy as np


class LinearTimeScaling:
    """Linear time-scaling: constant velocity."""
    def __init__(self, duration):
        self.duration = duration

    def eval(self, t):
        s = t / self.duration
        ds = np.ones_like(t) / self.duration
        dds = np.zeros_like(t)
        return s, ds, dds


class CubicTimeScaling:
    """Cubic time-scaling: zero velocity at end points."""
    def __init__(self, duration):
        self.coeffs = np.array([0, 0, 3.0 / duration**2, -2.0 / duration**3])

    def eval(self, t):
        s = self.coeffs.dot([np.ones_like(t), t, t**2, t**3])
        ds = self.coeffs[1:].dot([np.ones_like(t), 2*t, 3*t**2])
        dds = self.coeffs[2:].dot([2*np.ones_like(t), 6*t])
        return s, ds, dds


class QuinticTimeScaling:
    """Quintic time-scaling: zero velocity and acceleration at end points."""
    def __init__(self, duration):
        T = duration
        A = np.array([[1, 0, 0, 0, 0, 0],
                      [1, T, T**2, T**3, T**4, T**5],
                      [0, 1, 0, 0, 0, 0],
                      [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
                      [0, 0, 2, 0, 0, 0],
                      [0, 0, 2, 6*T, 12*T**2, 20*T**3]])
        b = np.array([0, 1, 0, 0, 0, 0])
        self.coeffs = np.linalg.solve(A, b)

    def eval(self, t):
        s = self.coeffs.dot([np.ones_like(t), t, t**2, t**3, t**4, t**5])
        ds = self.coeffs[1:].dot([np.ones_like(t), 2*t, 3*t**2, 4*t**3, 5*t**4])
        dds = self.coeffs[2:].dot([2*np.ones_like(t), 6*t, 12*t**2, 20*t**3])
        return s, ds, dds


def eval_trapezoidal(t, v, a, ta, T):
    """Evaluate trapezoidal time-scaling given:
        t:  evaluation times
        v:  cruise velocity
        a:  acceleration
        ta: time of acceleration
        T:  total duration
    """
    t1 = t < ta
    t2 = (t >= ta) & (t < T - ta)
    t3 = t >= T - ta

    # stage 1: acceleration
    s1 = 0.5*a*t**2
    ds1 = a*t
    dds1 = a * np.ones_like(t)

    # stage 2: constant velocity
    s2 = v*t - 0.5*v**2/a
    ds2 = v * np.ones_like(t)
    dds2 = np.zeros_like(t)

    # stage 3: deceleration
    s3 = v*T - v**2/a - 0.5*a*(t - T)**2
    ds3 = a*(T - t)
    dds3 = -a * np.ones_like(t)

    s = t1 * s1 + t2 * s2 + t3 * s3
    ds = t1 * ds1 + t2 * ds2 + t3 * ds3
    dds = t1 * dds1 + t2 * dds2 + t3 * dds3

    return s, ds, dds


class TrapezoidalTimeScalingV:
    """Trapezoidal time scaling specifying cruising velocity and duration."""
    def __init__(self, v, duration):
        assert (v * duration > 1 and v*duration <= 2), "Need 2 >= v*T > 1 for a 3-stage profile."
        self.v = v
        self.duration = duration
        self.a = v**2 / (v*duration - 1)
        self.ta = v / self.a

    def eval(self, t):
        return eval_trapezoidal(t, self.v, self.a, self.ta, self.duration)


class TrapezoidalTimeScalingA:
    """Trapezoidal time scaling specifying acceleration and duration."""
    def __init__(self, a, duration):
        assert a * duration**2 >= 4, "Need a*T**2 >= 4 to ensure motion is completed in time."
        self.v = 0.5*(a*duration - np.sqrt(a*(a*duration**2 - 4)))
        self.duration = duration
        self.a = a
        self.ta = self.v / a

    def eval(self, t):
        return eval_trapezoidal(t, self.v, self.a, self.ta, self.duration)

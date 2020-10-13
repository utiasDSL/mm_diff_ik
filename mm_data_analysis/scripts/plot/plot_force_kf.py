#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import mm_data_analysis.util as util
from mm_force_control.filter import ExponentialSmoother

import IPython


def main():
    bag = rosbag.Bag('../../bags/2020-02-25/compliance/compliance_wp0.bag')

    force_info_msgs = [msg for _, msg, _ in bag.read_messages('/force/info')]

    t = util.parse_time(force_info_msgs)

    force_raw = np.array([[msg.force_raw.x, msg.force_raw.y, msg.force_raw.z]
                          for msg in force_info_msgs])

    # Manual filtering to tune.
    smoother = ExponentialSmoother(tau=0.1, x0=np.zeros(3))
    force_filtered = np.zeros(force_raw.shape)
    for i in xrange(1, force_filtered.shape[0]):
        dt = t[i] - t[i-1]
        force_filtered[i, :] = smoother.next(force_raw[i, :], dt)

    # Kalman filter
    var_b0 = 1
    var_f0 = 1
    var_v = 0.01
    var_w = 1
    var_n = 100

    # var_n = 2000 makes it look like exp smoothing

    Q = np.diag([var_v, var_w])
    C = np.ones((1, 2))

    f_kf = np.zeros(force_raw.shape[0])
    b_kf = np.zeros(force_raw.shape[0])

    # Initial conditions and first iteration
    Pch = np.diag([var_b0, var_f0])
    xch = np.zeros((2, 1))

    K = Pch.dot(C.T) / (C.dot(Pch).dot(C.T) + var_n)
    P = (np.eye(2) - K.dot(C)).dot(Pch)
    y = force_raw[0, 0]
    x = xch + K.dot(y - C.dot(xch))

    b_kf[0] = x[0, 0]
    f_kf[0] = x[1, 0]

    for i in xrange(1, len(f_kf)):
        Pch = P + Q
        xch = x
        K = Pch.dot(C.T) / (C.dot(Pch).dot(C.T) + var_n)
        P = (np.eye(2) - K.dot(C)).dot(Pch)
        y = force_raw[i, 0]
        x = xch + K.dot(y - C.dot(xch))

        b_kf[i] = x[0, 0]
        f_kf[i] = x[1, 0]

    plt.figure(1)
    plt.grid()
    plt.plot(t, force_raw[:, 0], label='Raw')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Raw Force')

    plt.figure(2)
    plt.grid()
    plt.plot(t, force_filtered[:, 0], label='Exp. smoothed')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Exponential Smoothing')

    plt.figure(3)
    plt.grid()
    plt.plot(t, f_kf, label='Force')
    plt.plot(t, b_kf, label='Bias')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Kalman Filtering')

    plt.show()


if __name__ == '__main__':
    main()

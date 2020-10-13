#!/usr/bin/env python2
from __future__ import print_function

import time
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import mm_data_analysis.util as util

import IPython


def extract_xy(scan):
    n = len(scan.ranges)
    ranges = np.array(scan.ranges)
    angles = np.array([scan.angle_min + i*scan.angle_increment for i in range(n)])

    valid_idx = np.nonzero(np.logical_and(ranges >= scan.range_min, ranges <= scan.range_max))

    x = np.cos(angles) * ranges
    y = np.sin(angles) * ranges

    return x[valid_idx], y[valid_idx]


def fit_circle(x, y, N):
    ''' Least squares circle fit. '''
    # the problem is linear if we choose parameters
    # w = [xc, yc, xc**2 + yc**2 - r**2]

    A = np.zeros((3, 3))
    b = np.zeros(3)

    for i in range(N):
        alpha = np.array([-2*x[i], -2*y[i], 1])
        beta = (x[i]**2 + y[i]**2)
        b += beta * alpha
        A += np.outer(alpha, alpha)

    # normalize by number of data points
    A = A / N
    b = b / N

    params = np.linalg.solve(A, -b)

    xc = params[0]
    yc = params[1]
    r = np.sqrt(xc**2 + yc**2 - params[2])

    return xc, yc, r


def extract_barrel_pts(marker_msg):
    xs = np.zeros(5)
    ys = np.zeros(5)
    i = 0
    for marker in marker_msg.markers:
        if marker.subject_name == 'Barrel':
            xs[i] = marker.translation.x
            ys[i] = marker.translation.y
            i += 1
    return xs * 0.001, ys * 0.001


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    scan_msgs = [msg for _, msg, _ in bag.read_messages('/front/scan')]
    t = util.parse_time(scan_msgs)

    marker_msgs = [msg for _, msg, _ in bag.read_messages('/vicon/markers')]
    t2 = util.parse_time(marker_msgs)
    marker_msgs_aligned = util.align_lists(t, scan_msgs, t2, marker_msgs)

    # IPython.embed()

    x, y = extract_xy(scan_msgs[0])

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot([0], [0], 'o', color='r')
    # ax.set_xlim(-5, 5)
    # ax.set_ylim(-5, 5)
    ax.set_xlim(-1, 3)
    ax.set_ylim(-2, 2)
    text = ax.text(-4, 4, 't = 0s')
    ax.grid()
    lines, = ax.plot(x, y, 'o', color='b')
    # barrel, = ax.plot([], [], 'o', color='k')

    barrel = plt.Circle((0, 0), 0, color='k', fill=False)
    ax.add_patch(barrel)

    fig.canvas.draw()
    fig.canvas.flush_events()

    for i in xrange(1, len(scan_msgs)):
        dt = t[i] - t[i - 1]
        time.sleep(dt)
        text.set_text('t = {:.3f}s'.format(t[i]))
        x, y = extract_xy(scan_msgs[i])

        # TODO this needs to be relative to the base pose
        x += 0.393

        bx, by = extract_barrel_pts(marker_msgs_aligned[i])
        xc, yc, r = fit_circle(bx, by, 5)
        barrel.center = (xc, yc)
        barrel.radius = r
        # print(bx)
        # print(by)
        # barrel.set_xdata(bx)
        # barrel.set_ydata(by)

        lines.set_xdata(x)
        lines.set_ydata(y)
        fig.canvas.draw()
        fig.canvas.flush_events()

    IPython.embed()


if __name__ == '__main__':
    main()

#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import mm_data_analysis.util as util
from mm_force_control.filter import ExponentialSmoother

import IPython


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    force_info_msgs = [msg for _, msg, _ in bag.read_messages('/force/info')]
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]

    t = util.parse_time(force_info_msgs)

    force_raw = np.array([[msg.force_raw.x, msg.force_raw.y, msg.force_raw.z]
                          for msg in force_info_msgs])
    force_filtered = np.array([[msg.force_filtered.x, msg.force_filtered.y,
                                msg.force_filtered.z] for msg in force_info_msgs])
    force_world = np.array([[msg.force_world.x, msg.force_world.y, msg.force_world.z]
                            for msg in force_info_msgs])

    f_norms = np.linalg.norm(force_world, axis=1)
    nf = force_world / f_norms[:, None]

    # Manual filtering to tune.
    smoother = ExponentialSmoother(tau=0.5, x0=np.zeros(3))
    force_filtered2 = np.zeros(force_filtered.shape)
    for i in xrange(1, force_filtered2.shape[0]):
        dt = t[i] - t[i-1]
        force_filtered2[i, :] = smoother.next(force_raw[i, :], dt)

    plt.figure(1)
    plt.grid()
    plt.plot(t, force_raw[:, 0], label='fx')
    plt.plot(t, force_raw[:, 1], label='fy')
    plt.plot(t, force_raw[:, 2], label='fz')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Raw Force')

    plt.figure(2)
    plt.grid()
    plt.plot(t, force_filtered[:, 0], label='fx')
    plt.plot(t, force_filtered[:, 1], label='fy')
    plt.plot(t, force_filtered[:, 2], label='fz')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Filtered Force')

    plt.figure(3)
    plt.grid()
    plt.plot(t, nf[:, 0], label='fx')
    plt.plot(t, nf[:, 1], label='fy')
    plt.plot(t, nf[:, 2], label='fz')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('World Force')

    plt.figure(4)
    plt.grid()
    plt.plot(t, f_norms)
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('World Force Magnitude')

    # plt.figure(5)
    # plt.grid()
    # plt.plot(t, force_filtered2[:, 0], label='fx')
    # plt.plot(t, force_filtered2[:, 1], label='fy')
    # plt.plot(t, force_filtered2[:, 2], label='fz')
    # plt.plot(t, np.linalg.norm(force_filtered2, axis=1), label='norm', color='k')
    # plt.legend()
    # plt.xlabel('Time (s)')
    # plt.ylabel('Force (N)')
    # plt.title('Filtered Force 2')

    plt.show()


if __name__ == '__main__':
    main()

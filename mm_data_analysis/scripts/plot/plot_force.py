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

    wrench_info_msgs = [msg for _, msg, _ in bag.read_messages('/mm_wrench/info')]
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]

    t = util.parse_time(wrench_info_msgs)

    force_raw = np.array([[msg.raw.force.x, msg.raw.force.y, msg.raw.force.z]
                          for msg in wrench_info_msgs])
    force_filtered = np.array([[msg.filtered.force.x, msg.filtered.force.y,
                                msg.filtered.force.z] for msg in wrench_info_msgs])
    force_world = np.array([[msg.world.force.x, msg.world.force.y, msg.world.force.z]
                            for msg in wrench_info_msgs])

    force_desired = np.array([msg.force_desired for msg in wrench_info_msgs])
    # force_desired = -5 * np.ones_like(force_desired)

    # f_norms = np.linalg.norm(force_world, axis=1)
    # nf = force_world / f_norms[:, None]

    # Manual filtering to tune.
    # smoother = ExponentialSmoother(tau=0.5, x0=np.zeros(3))
    # force_filtered2 = np.zeros(force_filtered.shape)
    # for i in xrange(1, force_filtered2.shape[0]):
    #     dt = t[i] - t[i-1]
    #     force_filtered2[i, :] = smoother.next(force_raw[i, :], dt)

    plt.figure()
    plt.grid()
    plt.plot(t, force_raw[:, 0], label='fx')
    plt.plot(t, force_raw[:, 1], label='fy')
    plt.plot(t, force_raw[:, 2], label='fz')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Raw Force')

    plt.figure()
    plt.grid()
    plt.plot(t, force_filtered[:, 0], label='fx')
    plt.plot(t, force_filtered[:, 1], label='fy')
    plt.plot(t, force_filtered[:, 2], label='fz')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Filtered Force')

    plt.figure()
    plt.grid()
    plt.plot(t, force_world[:, 0], label='fx')
    plt.plot(t, force_world[:, 1], label='fy')
    plt.plot(t, force_world[:, 2], label='fz')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('World Force (filtered)')

    plt.figure()
    plt.grid()
    plt.plot(t, force_world[:, 2], label='Actual')
    plt.plot(t, force_desired, label='Desired')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Force in z-direction')

    plt.show()


if __name__ == '__main__':
    main()

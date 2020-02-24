#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import mm_data_analysis.util as util

import IPython


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    force_info_msgs = [msg for _, msg, _ in bag.read_messages('/force/info')]
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]

    aligned_msgs = util.align_msgs(force_info_msgs, pose_msgs)
    pose_msgs_aligned = [msg for _, msg in aligned_msgs]

    t0 = force_info_msgs[0].header.stamp.to_sec()
    t = util.parse_time(force_info_msgs)
    fs = np.array([[msg.force_world.x, msg.force_world.y, msg.force_world.z]
                   for msg in force_info_msgs])

    t2 = util.parse_time(pose_msgs, t0=t0)
    pos_msgs = [msg.error.position for msg in pose_msgs_aligned]
    ps = np.array([[msg.x, msg.y, msg.z] for msg in pos_msgs]) * 10
    ps = ps - ps[0, :]  # normalize to 0 initial position

    # IPython.embed()


    plt.figure(1)
    plt.grid()
    plt.plot(t, fs[:, 0], label='fx')
    # plt.plot(t, fs[:, 1], label='fy')
    # plt.plot(t, fs[:, 2], label='fz')
    plt.plot(t, ps[:, 0], label='px')
    # plt.plot(t, ps[:, 1], label='py')
    # plt.plot(t, ps[:, 2], label='pz')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('World Force')

    # plt.figure(2)
    # plt.grid()
    # plt.plot(tps, ps[:, 0], label='px')
    # plt.plot(tps, ps[:, 1], label='py')
    # plt.plot(tps, ps[:, 2], label='pz')
    # plt.legend()
    # plt.xlabel('Time (s)')
    # plt.ylabel('Position (m)')
    # plt.title('Positions')

    plt.show()


if __name__ == '__main__':
    main()

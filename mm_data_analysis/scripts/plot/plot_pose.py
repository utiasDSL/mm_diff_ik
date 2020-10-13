#!/usr/bin/env python2
from __future__ import print_function

import sys

import rosbag
import matplotlib.pyplot as plt
import mm_data_analysis.plot as mmplt


def main():
    bag = rosbag.Bag(sys.argv[1])
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    mmplt.plot_pose_actual_vs_desired(pose_msgs)
    plt.show()

    # bag1 = rosbag.Bag(sys.argv[1])
    # bag2 = rosbag.Bag(sys.argv[2])
    #
    # msgs1 = [msg for _, msg, _ in bag1.read_messages('/mm_pose_state')]
    # msgs2 = [msg for _, msg, _ in bag2.read_messages('/mm_pose_state')]
    #
    # mmplt.plot_qx_actual_vs_desired(msgs1, msgs2)
    # plt.show()


if __name__ == '__main__':
    main()

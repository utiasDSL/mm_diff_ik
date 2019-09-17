#!/usr/bin/env python2
from __future__ import print_function

import sys

import rosbag
import matplotlib.pyplot as plt
import mm_data_analysis.plot as mmplt


BAG = 'bags/2019-09-12/force/force_perturb2_2019-09-12-14-21-18.bag'


def main():
    bag = rosbag.Bag(BAG)
    force_state_msgs = [msg for _, msg, _ in bag.read_messages('/force_control/state')]
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]

    mmplt.plot_forces(force_state_msgs, pose_msgs)
    plt.show()


if __name__ == '__main__':
    main()

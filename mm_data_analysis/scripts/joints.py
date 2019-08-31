#!/usr/bin/env python2
from __future__ import print_function

import sys

import rosbag
import matplotlib.pyplot as plt
import mm_data_analysis.plot as mmplt

import IPython


def main():
    bag = rosbag.Bag(sys.argv[1])
    joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]

    mmplt.plot_joints(joint_msgs)
    #mmplt.plot_manipulability(joint_msgs)
    plt.show()

    # IPython.embed()


if __name__ == '__main__':
    main()

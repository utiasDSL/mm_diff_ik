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

    msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    t = util.parse_time(msgs)
    vals = np.array([msg.objective for msg in msgs])

    plt.figure()
    plt.plot(t, vals)
    plt.grid()
    plt.title('Objective Value')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.show()


if __name__ == '__main__':
    main()

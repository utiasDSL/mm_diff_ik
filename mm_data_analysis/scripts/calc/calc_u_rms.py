#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import mm_data_analysis.util as util


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]

    us = np.array([msg.u for msg in pose_msgs])
    u_norms = np.linalg.norm(us, axis=1)
    u_rms = util.rms(u_norms)

    print("RMS of ||u|| = {}".format(u_rms))
    print("Mean of ||u|| = {}".format(np.mean(u_norms)))


if __name__ == '__main__':
    main()

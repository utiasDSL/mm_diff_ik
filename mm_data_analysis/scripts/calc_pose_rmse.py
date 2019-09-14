#!/usr/bin/env python2
from __future__ import print_function

import sys
import numpy as np
import rosbag

import mm_data_analysis.util as util


def main():
    bag = rosbag.Bag(sys.argv[1])
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]

    err_msgs = [msg.error.position for msg in pose_msgs]
    errs = util.vec3_msg_to_np(err_msgs)
    norms = np.linalg.norm(errs, axis=1)
    rmse = util.rms(norms)

    print('RMSE = {} m'.format(rmse))


if __name__ == '__main__':
    main()

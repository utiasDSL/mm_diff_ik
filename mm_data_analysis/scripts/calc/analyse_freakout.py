#!/usr/bin/env python2
import numpy as np
import rosbag
import mm_kinematics.kinematics as kinematics
import mm_data_analysis.util as util

import IPython


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)
    msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]

    qs = np.array([msg.q for msg in msgs])
    dqs = np.array([msg.dq for msg in msgs])
    us = np.array([msg.u for msg in msgs])

    objs = np.array([msg.objective for msg in msgs])
    statuses = np.array([msg.status for msg in msgs])

    # find bad indices by looking where the angular velocity of the base
    # exceeds 1 rad/s
    # bad_idx, = np.nonzero(np.abs(us[:, 2]) > 0.45)
    # bad_idx, = np.nonzero(np.isnan(objs))
    bad_idx, = np.nonzero(statuses != 0)

    print('u = {}'.format(us[bad_idx[0], :]))
    print('q = {}'.format(qs[bad_idx[0], :]))
    print('dq = {}'.format(dqs[bad_idx[0], :]))

    IPython.embed()


if __name__ == '__main__':
    main()

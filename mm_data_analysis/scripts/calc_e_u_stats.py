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

    # inputs
    us = np.array([msg.u for msg in msgs])

    # position errors
    pos_errs = np.array([[msg.error.position.x, msg.error.position.y,
                          msg.error.position.z] for msg in msgs])

    rot_errs = np.array([[msg.rotation_error.x, msg.rotation_error.y,
                          msg.rotation_error.z] for msg in msgs])

    u_max = np.max(np.abs(us), axis=0)
    u_mean = np.mean(np.abs(us), axis=0)

    pos_err_max = np.max(np.abs(pos_errs), axis=0)
    pos_err_mean = np.mean(np.abs(pos_errs), axis=0)

    rot_err_max = np.max(np.abs(rot_errs), axis=0)
    rot_err_mean = np.mean(np.abs(rot_errs), axis=0)

    print('max u = {}'.format(u_max))
    print('mean u = {}'.format(u_mean))

    print('max pos err = {}'.format(pos_err_max))
    print('mean pos err = {}'.format(pos_err_mean))

    print('max rot err = {}'.format(rot_err_max))
    print('mean rot err = {}'.format(rot_err_mean))


if __name__ == '__main__':
    main()

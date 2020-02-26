#!/usr/bin/env python2
from __future__ import print_function

import numpy as np
import rosbag
import tf.transformations as tfs

import mm_data_analysis.util as util

import IPython


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]

    err_msgs = [msg.error.position for msg in pose_msgs]
    errs = util.vec3_msg_to_np(err_msgs)
    norms = np.linalg.norm(errs, axis=1)
    rmse = util.rms(norms) * 1000.0  # convert to mm

    rot_err_msgs = [msg.error.orientation for msg in pose_msgs]
    quats = np.array([[msg.x, msg.y, msg.z, msg.w] for msg in rot_err_msgs])

    # euler_errs = np.zeros((len(quats), 3))
    # for i in xrange(len(quats)):
    #     euler_errs[i, :] = tfs.euler_from_quaternion(quats[i, :])
    # euler_norms = np.linalg.norm(euler_errs, axis=1)
    # euler_rmse = util.rms(euler_norms)
    rot_errs = np.zeros(len(quats))
    for i in xrange(len(quats)):
        eps = np.linalg.norm(quats[i, :3])
        eta = quats[i, 3]
        rot_errs[i] = 2 * np.arctan2(eps, eta)
    rot_rmse = util.rms(rot_errs)

    print('Position RMSE = {} mm'.format(rmse))
    print('Rotation RMSE = {} rad'.format(rot_rmse))


if __name__ == '__main__':
    main()

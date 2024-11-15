#!/usr/bin/env python2
"""New-style pose error calculation."""
from __future__ import print_function

import numpy as np
import rosbag
import tf.transformations as tfs

from mm_kinematics import KinematicModel
import mm_data_analysis.util as util

import IPython


def rmse_from_control_state(control_msgs):
    """Calculate error reported from pose state messages."""
    p_errs = util.vec3_list_to_np([msg.error.pose.position for msg in control_msgs])
    p_err_norms = np.linalg.norm(p_errs, axis=1)
    p_rmse = util.rms(p_err_norms) * 1000.0  # convert to mm

    quat_errs = [util.quat_to_np(msg.error.pose.orientation) for msg in control_msgs]
    angle_errs = np.array([util.quat_angle(quat) for quat in quat_errs])
    angle_rmse = util.rms(angle_errs)

    return p_rmse, angle_rmse


def rmse_from_joint_state(control_msgs, joint_msgs):
    """Calculate error based on ground truth reported from joint messages and
       desired pose from the controller pose state messages."""
    model = KinematicModel()

    t0 = util.msg_time(control_msgs[0])

    joint_msgs_traj = util.trim_to_traj(joint_msgs, control_msgs)
    Ts = [model.calc_T_w_tool(msg.position) for msg in joint_msgs_traj]

    t_pose = util.parse_time(control_msgs)
    t_joint = util.parse_time(joint_msgs_traj, t0=t0)

    # ps = np.array([T[:3, 3] for T in Ts])
    ps = [T[:3, 3] for T in Ts]
    ps = util.linear_interpolate_list(t_pose, t_joint, ps)

    # quats = np.array([tfs.quaternion_from_matrix(T) for T in Ts])
    quats = [tfs.quaternion_from_matrix(T) for T in Ts]
    quats = util.spherical_interpolate_quaternions(t_pose, t_joint, quats)

    pds = np.array([util.vec3_to_np(msg.desired.pose.position) for msg in control_msgs])
    # pds = util.linear_interpolate_list(t_joint, t_pose, pds)

    quatds = [util.quat_to_np(msg.desired.pose.orientation) for msg in control_msgs]
    # quatds = util.spherical_interpolate_quaternions(t_joint, t_pose, quatds)

    p_errs = pds - ps
    p_err_norms = np.linalg.norm(p_errs, axis=1)
    p_rmse = util.rms(p_err_norms) * 1000.0
    p_mae = np.mean(p_err_norms) * 1000.0

    quat_errs = [util.quat_error(quatd, quat) for quatd, quat in zip(quatds, quats)]
    angle_errs = np.array([util.quat_angle(quat) for quat in quat_errs])
    angle_rmse = util.rms(angle_errs)
    angle_mae = np.mean(np.abs(angle_errs))

    return p_rmse, angle_rmse, p_mae, angle_mae


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    control_msgs = [msg for _, msg, _ in bag.read_messages('/mm/control/cartesian/info')]
    joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm/joint_states')]

    p_rmse1, angle_rmse1 = rmse_from_control_state(control_msgs)
    print('Position RMSE 1 = {} mm'.format(p_rmse1))
    print('Rotation RMSE 1 = {} rad'.format(angle_rmse1))

    p_rmse2, angle_rmse2, p_mae, angle_mae = rmse_from_joint_state(control_msgs, joint_msgs)
    print('Position RMSE 2 = {} mm'.format(p_rmse2))
    print('Rotation RMSE 2 = {} rad'.format(angle_rmse2))
    print('Position MAE 2 = {} mm'.format(p_mae))
    print('Rotation MAE 2 = {} rad'.format(angle_mae))


if __name__ == '__main__':
    main()

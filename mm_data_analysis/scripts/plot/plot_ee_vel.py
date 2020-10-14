#!/usr/bin/env python2
import numpy as np
import sys
import rosbag
import mm_data_analysis.util as util
import tf.transformations as tfs
import mm_kinematics.kinematics as kinematics
import matplotlib.pyplot as plt

import IPython


def numerical_diff(p, t):
    dt = t[1:] - t[:-1]
    v = (p[1:] - p[:-1]) / dt[:, None]
    return v


def smooth(a):
    a_smooth = np.zeros_like(a)
    for i in range(a.shape[0]):
        if i == 0:
            a_smooth[i, :] = (a[i,:] + a[i+1,:]) / 2.0
        elif i == a.shape[0] - 1:
            a_smooth[i, :] = (a[i-1,:] + a[i,:]) / 2.0
        else:
            a_smooth[i, :] = (a[i-1,:] + a[i,:] + a[i+1,:]) / 3.0
    return a_smooth


def Rz(yaw):
    c = np.cos(yaw)
    s = np.sin(yaw)
    return np.array([[c, -s,  0,],
                     [s,  c, 0,],
                     [0,  0, 1]])


def main():
    bag = rosbag.Bag(sys.argv[1])

    # parse ee values
    ee_vicon_msgs = [msg for _, msg, _ in bag.read_messages('/vicon/ThingEE/ThingEE')]
    pee = np.array([[msg.transform.translation.x, msg.transform.translation.y,
                    msg.transform.translation.z] for msg in ee_vicon_msgs])
    t0 = util.parse_t0(ee_vicon_msgs)
    tee = util.parse_time(ee_vicon_msgs)
    vee = numerical_diff(pee, tee)

    # TODO joint velocities are in messages
    joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]
    vee_model = np.zeros((len(joint_msgs), 3))
    pee_model = np.zeros((len(joint_msgs), 3))
    yaws = np.zeros(len(joint_msgs))
    tq = util.parse_time(joint_msgs, t0=t0)
    for idx, msg in enumerate(joint_msgs):
        q = np.array(msg.position)
        w_T_ee = kinematics.calc_w_T_palm(q)
        yaws[idx] = q[2]
        pee_model[idx, :] = w_T_ee[:3,  3]
        dq = np.array(msg.velocity)
        J = kinematics.jacobian(q)
        vee_model[idx, :] = J.dot(dq)[:3]

    yaws_aligned = np.array(util.align_lists(tee, pee, tq, yaws))
    pee_model_aligned = np.array(util.align_lists(tee, pee, tq, pee_model))
    pee_err = pee - pee_model_aligned

    # rotate into the base frame
    pee_err_rot = np.zeros_like(pee_err)
    for idx in range(yaws_aligned.shape[0]):
        w_R_b = Rz(yaws_aligned[idx])
        pee_err_rot[idx, :] = w_R_b.T.dot(pee_err[idx, :])

    pee_err_norm = np.linalg.norm(pee_err_rot, axis=1)

    vee_model_aligned = np.array(util.align_lists(tee, vee, tq, vee_model))
    vee_err = vee - vee_model_aligned

    # parse base values
    base_vicon_msgs = [msg for _, msg, _ in bag.read_messages('/vicon/ThingBase2/ThingBase2')]
    qb = np.zeros((len(base_vicon_msgs), 3))
    for idx, msg in enumerate(base_vicon_msgs):
        quat = np.array([msg.transform.rotation.x, msg.transform.rotation.y,
                         msg.transform.rotation.z, msg.transform.rotation.w])
        rpy = tfs.euler_from_quaternion(quat)
        yaw = rpy[2]
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        qb[idx, :] = [x, y, yaw]
    tb = util.parse_time(base_vicon_msgs, t0=t0)
    vb = numerical_diff(qb, tb)
    vb_smooth = smooth(vb)

    cmd_vel_topic = '/ridgeback_velocity_controller/cmd_vel'
    cmd_vel_msgs = [msg for _, msg, _ in bag.read_messages(cmd_vel_topic)]
    cmds = np.array([[msg.linear.x, msg.linear.y, msg.angular.z] for msg in cmd_vel_msgs])
    cmd_vel_times = np.array([t.to_sec() for _, _, t in bag.read_messages(cmd_vel_topic)])
    tcmd = cmd_vel_times - t0

    cmds_aligned = np.array(util.align_lists(tb, vb, tcmd, cmds))

    vb_err = cmds_aligned - vb

    idx1 = 0
    idx2 = 0
    for i in xrange(len(tee)):
        if tee[i] > tcmd[0]:
            idx1 = i
            break
    for i in xrange(len(tee)):
        if tee[i] > tcmd[-1]:
            idx2 = i
            break

    pee_err_norm_inside = pee_err_norm[idx1:idx2] * 1000
    pee_err_mean = np.mean(pee_err_norm_inside)
    pee_err_var = np.var(pee_err_norm_inside)
    print('mean = {}'.format(pee_err_mean))
    print('var  = {}'.format(pee_err_var))


    # plt.figure()
    # plt.grid()
    # plt.plot(tee[:-1], vee[:, 0], label='vx')
    # plt.plot(tee[:-1], vee[:, 1], label='vy')
    # plt.plot(tee[:-1], vee[:, 2], label='vz')
    # plt.xlabel('Time (s)')
    # plt.ylabel('EE Velocity (m/s)')
    # plt.legend()
    # plt.title('EE Velocity')

    # plt.figure()
    # plt.grid()
    # plt.plot(tee, pee[:, 0], label='px')
    # plt.plot(tee, pee[:, 1], label='py')
    # plt.plot(tee, pee[:, 2], label='pz')
    # plt.xlabel('Time (s)')
    # plt.ylabel('EE Position (m)')
    # plt.legend()
    # plt.title('Actual EE Position')

    plt.figure()
    plt.grid()
    plt.plot(tee, pee_err_rot[:, 0] * 1000, label='px')
    plt.plot(tee, pee_err_rot[:, 1] * 1000, label='py')
    plt.plot(tee, pee_err_rot[:, 2] * 1000, label='pz')
    plt.xlabel('Time (s)')
    plt.ylabel('EE Position Error (mm)')
    plt.legend()
    plt.axvline(tcmd[0], color='k', linestyle='--')
    plt.axvline(tcmd[-1], color='k', linestyle='--')
    plt.title('EE Position Error')

    plt.figure()
    plt.grid()
    plt.plot(tee, pee_err_norm * 1000)
    plt.xlabel('Time (s)')
    plt.ylabel('EE Position Error (mm)')
    plt.title('EE Position Error Norm')
    plt.axvline(tcmd[0], color='k', linestyle='--')
    plt.axvline(tcmd[-1], color='k', linestyle='--')

    plt.figure()
    plt.grid()
    plt.plot(tb[:-1], vb_smooth[:, 0], label='vx (m/s)')
    plt.plot(tb[:-1], vb_smooth[:, 1], label='vy (m/s)')
    plt.plot(tb[:-1], vb_smooth[:, 2], label='wz (rad/s)')
    plt.xlabel('Time (s)')
    plt.ylabel('Base Velocity')
    plt.legend()
    plt.title('Actual Base Velocity')

    # plt.figure()
    # plt.grid()
    # plt.plot(tcmd, cmds[:, 0], label='vx (m/s)')
    # plt.plot(tcmd, cmds[:, 1], label='vy (m/s)')
    # plt.plot(tcmd, cmds[:, 2], label='wz (rad/s)')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Commanded Velocity')
    # plt.legend()
    # plt.title('Commanded Base Velocity')
    #
    # plt.figure()
    # plt.grid()
    # plt.plot(tb[:-1], vb_err[:, 0], label='vx (m/s)')
    # plt.plot(tb[:-1], vb_err[:, 1], label='vy (m/s)')
    # plt.plot(tb[:-1], vb_err[:, 2], label='wz (rad/s)')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Velocity Error')
    # plt.legend()
    # plt.title('Base Velocity Error')

    # plt.figure()
    # plt.grid()
    # plt.plot(tq, vee_model[:, 0], label='vx')
    # plt.plot(tq, vee_model[:, 1], label='vy')
    # plt.plot(tq, vee_model[:, 2], label='vz')
    # plt.xlabel('Time (s)')
    # plt.ylabel('EE Velocity (m/s)')
    # plt.legend()
    # plt.title('EE Predicted Velocity')
    #
    # plt.figure()
    # plt.grid()
    # plt.plot(tee[:-1], vee_err[:, 0], label='vx')
    # plt.plot(tee[:-1], vee_err[:, 1], label='vy')
    # plt.plot(tee[:-1], vee_err[:, 2], label='vz')
    # plt.xlabel('Time (s)')
    # plt.ylabel('EE Velocity (m/s)')
    # plt.legend()
    # plt.title('EE Velocity Error')

    plt.show()


if __name__ == '__main__':
    main()

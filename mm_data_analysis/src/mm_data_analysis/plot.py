#!/usr/bin/env python2
from __future__ import print_function

import matplotlib.pyplot as plt
import numpy as np

from mm_data_analysis.util import parse_time, vec3_msg_to_np, trim_to_traj

import IPython


JOINT_NAMES = ['qx', 'qy', 'qt', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6']


def plot_pose_error(pose_msgs):
    t = parse_time(pose_msgs)
    pos_errs = [msg.error.position for msg in pose_msgs]
    quat_errs = [msg.error.orientation for msg in pose_msgs]

    px = [p.x for p in pos_errs]
    py = [p.y for p in pos_errs]
    pz = [p.z for p in pos_errs]

    qx = [q.x for q in quat_errs]
    qy = [q.y for q in quat_errs]
    qz = [q.z for q in quat_errs]

    plt.figure()
    plt.plot(t, px, label='x')
    plt.plot(t, py, label='y')
    plt.plot(t, pz, label='z')
    plt.grid()
    plt.legend()
    plt.title('End Effector Translation Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')

    plt.figure()
    plt.plot(t, qx, label='x')
    plt.plot(t, qy, label='y')
    plt.plot(t, qz, label='z')
    plt.grid()
    plt.legend()
    plt.title('End Effector Rotation Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (rad)')


# Function developed to highlight the difference between using orientation
# control and not.
def plot_qx_actual_vs_desired(pose_msgs1, pose_msgs2):
    t1 = parse_time(pose_msgs1)
    t2 = parse_time(pose_msgs2)

    # same for both
    qd = [msg.desired.orientation for msg in pose_msgs1]
    qxd = [q.x for q in qd]

    q1 = [msg.actual.orientation for msg in pose_msgs1]
    qx1 = [q.x if q.w >= 0 else -q.x for q in q1]

    q2 = [msg.actual.orientation for msg in pose_msgs2]
    qx2 = [q.x if q.w >= 0 else -q.x for q in q2]

    plt.figure(figsize=(10, 3.5))
    plt.plot(t1, qxd, label='$q_x^{(d)}$', linewidth=2)
    plt.plot(t1, qx1, label='$q_x^{(1)}$', linewidth=2)
    plt.plot(t2, qx2, label='$q_x^{(2)}$', linewidth=2)
    plt.grid()
    plt.legend()
    plt.title('End Effector Rotation')
    plt.xlabel('Time (s)')
    plt.ylabel('Rotation')


def plot_pose_actual_vs_desired(pose_msgs):
    t = parse_time(pose_msgs)
    pds = [msg.desired.position for msg in pose_msgs]
    qds = [msg.desired.orientation for msg in pose_msgs]
    pas = [msg.actual.position for msg in pose_msgs]
    qas = [msg.actual.orientation for msg in pose_msgs]

    pdx = [p.x for p in pds]
    pdy = [p.y for p in pds]
    pdz = [p.z for p in pds]

    qdx = [q.x for q in qds]
    qdy = [q.y for q in qds]
    qdz = [q.z for q in qds]

    pax = [p.x for p in pas]
    pay = [p.y for p in pas]
    paz = [p.z for p in pas]

    qax = [q.x if q.w >= 0 else -q.x for q in qas]
    qay = [q.y if q.w >= 0 else -q.y for q in qas]
    qaz = [q.z if q.w >= 0 else -q.z for q in qas]

    plt.figure()
    plt.plot(t, pdx, 'r', label='$x_d$')
    plt.plot(t, pdy, 'g', label='$y_d$')
    plt.plot(t, pdz, 'b', label='$z_d$')

    plt.plot(t, pax, 'r--', label='$x_a$')
    plt.plot(t, pay, 'g--', label='$y_a$')
    plt.plot(t, paz, 'b--', label='$z_a$')


    plt.grid()
    plt.legend()
    plt.title('End Effector Actual and Desired Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')

    vx = [msg.twist_desired.linear.x for msg in pose_msgs]
    vy = [msg.twist_desired.linear.y for msg in pose_msgs]
    vz = [msg.twist_desired.linear.z for msg in pose_msgs]

    # plt.figure()
    # plt.title('Feedforward velocity')
    # plt.plot(t, vx, label='vx')
    # plt.plot(t, vy, label='vy')
    # plt.plot(t, vz, label='vz')
    # plt.legend()

    plt.figure()
    plt.plot(t, qdx, 'r', label='$q^d_x$')
    plt.plot(t, qdy, 'g', label='$q^d_y$')
    plt.plot(t, qdz, 'b', label='$q^d_z$')

    plt.plot(t, qax, 'r--', label='$q^a_x$')
    plt.plot(t, qay, 'g--', label='$q^a_y$')
    plt.plot(t, qaz, 'b--', label='$q^a_z$')

    plt.grid()
    plt.legend()
    plt.title('End Effector Actual and Desired Orientation')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (rad)')


def plot_joints(mm_joint_states_msgs, idx=range(9)):
    t = parse_time(mm_joint_states_msgs)
    qs = np.array([msg.position for msg in mm_joint_states_msgs])

    plt.figure()

    for i in idx:
        plt.plot(t, qs[:,i], label=JOINT_NAMES[i])

    plt.grid()
    plt.legend()
    plt.title('Joint Positions')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')


def plot_joint_magnitudes(mm_joint_states_msgs):
    # TODO I want to look at magnitude of base vs arm joint movement here
    # this is effectively the velocity
    pass


def plot_force_raw_vs_filtered(force_state_msgs):
    t = parse_time(force_state_msgs)

    f_raw = vec3_msg_to_np([msg.force_raw for msg in force_state_msgs])
    f_filt = vec3_msg_to_np([msg.force_filtered for msg in force_state_msgs])
    f_world = vec3_msg_to_np([msg.force_world for msg in force_state_msgs])
    p_off = vec3_msg_to_np([msg.position_offset for msg in force_state_msgs])

    plt.figure()
    plt.subplot(311)
    plt.title('Force in x')
    plt.plot(t, f_raw[:,0], label='Raw')
    plt.plot(t, f_filt[:,0], label='Filtered')
    plt.grid()
    plt.legend()

    plt.subplot(312)
    plt.title('Force in y')
    plt.plot(t, f_raw[:,1], label='Raw')
    plt.plot(t, f_filt[:,1], label='Filtered')
    plt.grid()
    plt.legend()

    plt.subplot(313)
    plt.title('Force in z')
    plt.plot(t, f_raw[:,2], label='Raw')
    plt.plot(t, f_filt[:,2], label='Filtered')
    plt.grid()
    plt.legend()

    plt.xlabel('Time (s)')


def plot_forces(force_state_msgs, pose_msgs):
    force_state_msgs = trim_to_traj(force_state_msgs, pose_msgs)
    t_force = parse_time(force_state_msgs)
    f = vec3_msg_to_np([msg.force_world for msg in force_state_msgs])

    fig = plt.figure(figsize=(3.25, 2.1))
    plt.rcParams.update({'font.size': 8,
                         'text.usetex': True,
                         'legend.fontsize': 8})

    plt.subplot(211)
    ax = plt.gca()

    plt.plot(t_force, f[:,0], label='$f_x$', c='r', linewidth=1)
    plt.plot(t_force, f[:,1], label='$f_y$', c='b', linewidth=1)
    plt.plot(t_force, f[:,2], label='$f_z$', c='g', linewidth=1)
    plt.plot([t_force[0], t_force[-1]], [5, 5], color='k', linestyle='dashed', linewidth=1)
    plt.plot([t_force[0], t_force[-1]], [-5, -5], color='k', linestyle='dashed', linewidth=1)

    plt.ylabel('Force (N)', labelpad=0)
    ax.set_xticklabels([])
    plt.yticks([-10, 0, 10, 20])
    plt.legend(labelspacing=0, borderpad=0.3, loc=2)

    t_pose = parse_time(pose_msgs)
    pds = vec3_msg_to_np([msg.desired.position for msg in pose_msgs])
    # pes = vec3_msg_to_np([msg.error.position for msg in pose_msgs])

    x0 = pds[0,0]
    xf = pds[-1,0] - force_state_msgs[-1].position_offset.x
    y0 = pds[0,1]
    z0 = pds[0,2]

    plt.subplot(212)

    plt.plot(t_pose, pds[:,0], label='$x$', c='r', linewidth=1)
    plt.plot(t_pose, pds[:,1], label='$y$', c='b', linewidth=1)
    plt.plot(t_pose, pds[:,2], label='$z$', c='g', linewidth=1)

    plt.plot([t_pose[0], t_pose[-1]], [x0, xf], c=(0.25, 0, 0, 1), linestyle='dashed', linewidth=1)
    plt.plot([t_pose[0], t_pose[-1]], [y0, y0], c=(0, 0, 0.25, 1), linestyle='dashed', linewidth=1)
    plt.plot([t_pose[0], t_pose[-1]], [z0, z0], c=(0, 0.25, 0, 1), linestyle='dashed', linewidth=1)

    plt.legend(labelspacing=0, borderpad=0.3, loc=0)
    plt.ylabel('Desired position (m)', labelpad=11)
    plt.yticks([0, 1, 2, 3])

    # plt.subplot(313)
    # plt.plot(t_pose, pes[:,0], label='$x$', c='r', linewidth=2)
    # plt.plot(t_pose, pes[:,1], label='$y$', c='b', linewidth=2)
    # plt.plot(t_pose, pes[:,2], label='$z$', c='g', linewidth=2)
    #
    plt.xlabel('Time (s)')

    fig.tight_layout(pad=0.1)
    fig.savefig('force.pdf')


def plot_final(force_state_msgs, joint_state_msgs, obs_msgs, pose_msgs):
    force_state_msgs = trim_to_traj(force_state_msgs, pose_msgs)
    t_force = parse_time(force_state_msgs)
    f = vec3_msg_to_np([msg.force_world for msg in force_state_msgs])

    fig = plt.figure(figsize=(3.25, 3))
    plt.rcParams.update({'font.size': 8,
                         'text.usetex': True,
                         'legend.fontsize': 8})

    plt.subplot(211)
    ax = plt.gca()
    plt.plot(t_force, f[:,0], label='$f_x$', c='r', linewidth=1)
    plt.plot(t_force, f[:,1], label='$f_y$', c='b', linewidth=1)
    plt.plot(t_force, f[:,2], label='$f_z$', c='g', linewidth=1)
    plt.plot([t_force[0], t_force[-1]], [5, 5], color='k', linestyle='dashed', linewidth=1)
    plt.plot([t_force[0], t_force[-1]], [-5, -5], color='k', linestyle='dashed', linewidth=1)
    plt.ylabel('Force (N)', labelpad=0)
    # plt.xticks([])
    ax.set_xticklabels([])
    plt.yticks([-10, 0, 10, 20])
    plt.legend(labelspacing=0, borderpad=0.3, loc=2)

    t_pose = parse_time(pose_msgs)
    pas = vec3_msg_to_np([msg.error.position for msg in pose_msgs])
    # pes = vec3_msg_to_np([msg.error.position for msg in pose_msgs])

    plt.subplot(212)

    plt.plot(t_pose, pas[:,0], label='$x$', c='r', linewidth=1)
    plt.plot(t_pose, pas[:,1], label='$y$', c='b', linewidth=1)
    plt.plot(t_pose, pas[:,2], label='$z$', c='g', linewidth=1)

    plt.legend(labelspacing=0, borderpad=0.3, loc=0)
    plt.ylabel('End Effector position (m)')
    plt.yticks([0, 1, 2, 3])

    # plt.subplot(313)
    # plt.plot(t_pose, pes[:,0], label='$x$', c='r', linewidth=2)
    # plt.plot(t_pose, pes[:,1], label='$y$', c='b', linewidth=2)
    # plt.plot(t_pose, pes[:,2], label='$z$', c='g', linewidth=2)
    #
    plt.xlabel('Time (s)')

    fig.tight_layout(pad=0.1)
    fig.savefig('force.pdf')

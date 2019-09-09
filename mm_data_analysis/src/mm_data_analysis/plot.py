#!/usr/bin/env python2
from __future__ import print_function

import matplotlib.pyplot as plt
import numpy as np

import mm_kinematics.kinematics as kinematics
from mm_data_analysis.util import parse_time, vec3_msg_to_np

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
    plt.title('Linear End Effector Pose Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')

    plt.figure()
    plt.plot(t, qx, label='x')
    plt.plot(t, qy, label='y')
    plt.plot(t, qz, label='z')
    plt.grid()
    plt.legend()
    plt.title('Rotational End Effector Pose Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (rad)')


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

    qax = [q.x for q in qas]
    qay = [q.y for q in qas]
    qaz = [q.z for q in qas]

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

    plt.figure()
    plt.plot(t, qdx, label='x')
    plt.plot(t, qdy, label='y')
    plt.plot(t, qdz, label='z')
    plt.grid()
    plt.legend()
    plt.title('Rotational End Effector Desired Pose')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (rad)')


def plot_manipulability(mm_joint_states_msgs):
    t = parse_time(mm_joint_states_msgs)
    qs = np.array([msg.position for msg in mm_joint_states_msgs])

    # calculate MI at each timestep
    mi = np.zeros(len(t))
    for i in xrange(len(mi)):
        mi[i] = kinematics.manipulability(qs[i,:])

    plt.figure()
    plt.plot(t, mi)
    plt.grid()
    plt.legend()
    plt.title('Manipulability Index')
    plt.xlabel('Time (s)')
    plt.ylabel('Manipulability Index')


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


def plot_force_state(force_state_msgs):
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

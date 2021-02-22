#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mm_kinematics import JOINT_NAMES
import mm_data_analysis.util as util

import IPython


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    t0 = util.parse_t0(pose_msgs)
    t = util.parse_time(pose_msgs)
    tf = t[-1]
    qs = np.array([msg.q for msg in pose_msgs])
    dqs = np.array([msg.dq for msg in pose_msgs])
    us = np.array([msg.u for msg in pose_msgs])

    # messages directly from the source
    mm_joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]
    mm_joint_msgs_traj = util.trim_to_traj(mm_joint_msgs, pose_msgs)
    t2 = util.parse_time(mm_joint_msgs_traj, t0=t0)
    qs2 = np.array([msg.position for msg in mm_joint_msgs_traj])
    dqs2 = np.array([msg.velocity for msg in mm_joint_msgs_traj])

    ur_joint_msgs = [msg for _, msg, _ in bag.read_messages('/ur10_joint_states')]
    ur_joint_msgs_traj = util.trim_to_traj(ur_joint_msgs, pose_msgs)
    t3 = util.parse_time(ur_joint_msgs_traj, t0=t0)
    qs3 = np.array([msg.position for msg in ur_joint_msgs_traj])
    dqs3 = np.array([msg.velocity for msg in ur_joint_msgs_traj])

    # only take the ones during the trajectory: the ones when the
    # /mm_pose_state messages are also being published
    # msgs2_in_traj = (t2 >= 0) & (t2 <= tf)
    # t2 = t2[msgs2_in_traj]
    # qs2 = qs2[msgs2_in_traj, :]
    # dqs2 = dqs2[msgs2_in_traj, :]

    plt.figure()
    for i in xrange(9):
        plt.plot(t, qs[:, i], label=JOINT_NAMES[i])
    plt.grid()
    plt.legend()
    plt.title('Joint Positions')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')

    plt.figure()
    for i in xrange(9):
        plt.plot(t, dqs[:, i], label=JOINT_NAMES[i])
    plt.grid()
    plt.legend()
    plt.title('Joint Velocities')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular velocity (rad/s)')

    plt.figure()
    for i in xrange(9):
        plt.plot(t, us[:, i], label=JOINT_NAMES[i])
    plt.grid()
    plt.legend()
    plt.title('Joint Commands')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular velocity (rad/s)')

    plt.figure()
    for i in xrange(9):
        plt.plot(t2, qs2[:, i], label=JOINT_NAMES[i])
    plt.grid()
    plt.legend()
    plt.title('Joint Positions (/mm_joint_states)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')

    plt.figure()
    for i in xrange(6):
        plt.plot(t3, qs3[:, i], label=JOINT_NAMES[i+3])
    plt.grid()
    plt.legend()
    plt.title('Joint Positions (/ur10_joint_states)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')

    plt.figure()
    for i in xrange(6):
        plt.plot(t3, dqs3[:, i], label=JOINT_NAMES[i+3])
    plt.grid()
    plt.legend()
    plt.title('Joint Velocities (/ur10_joint_states)')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (rad/s)')

    # plt.figure(3)
    # for i in xrange(9):
    #     plt.plot(t, ddqs[:, i], label=JOINT_NAMES[i])
    # plt.grid()
    # plt.legend()
    # plt.title('Joint Accelerations')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Angular acceleration (rad/s^2)')

    plt.show()


if __name__ == '__main__':
    main()

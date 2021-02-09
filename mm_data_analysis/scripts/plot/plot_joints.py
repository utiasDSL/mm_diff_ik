#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import mm_data_analysis.util as util

import IPython


JOINT_NAMES = ['qx', 'qy', 'qt', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6']


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    # msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]
    # t = util.parse_time(msgs)
    # qs = np.array([msg.position for msg in msgs])
    # dqs = np.array([msg.velocity for msg in msgs])
    # ddqs = np.array([msg.effort for msg in msgs])

    msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    t = util.parse_time(msgs)
    qs = np.array([msg.q for msg in msgs])
    dqs = np.array([msg.dq for msg in msgs])
    us = np.array([msg.u for msg in msgs])

    plt.figure(1)
    for i in xrange(9):
        plt.plot(t, qs[:, i], label=JOINT_NAMES[i])
    plt.grid()
    plt.legend()
    plt.title('Joint Positions')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.figure(1)

    plt.figure(2)
    for i in xrange(9):
        plt.plot(t, dqs[:, i], label=JOINT_NAMES[i])
    plt.grid()
    plt.legend()
    plt.title('Joint Velocities')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular velocity (rad/s)')

    plt.figure(3)
    for i in xrange(9):
        plt.plot(t, us[:, i], label=JOINT_NAMES[i])
    plt.grid()
    plt.legend()
    plt.title('Joint Commands')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular velocity (rad/s)')

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

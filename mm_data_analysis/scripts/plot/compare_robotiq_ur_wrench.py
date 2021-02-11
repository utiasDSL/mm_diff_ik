#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import mm_data_analysis.util as util

import IPython


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    robotiq_msgs = [msg for _, msg, _ in bag.read_messages('/robotiq_force_torque_wrench')]
    ur_msgs = [msg for _, msg, _ in bag.read_messages('/wrench')]

    robotiq_t = util.parse_time(robotiq_msgs)
    ur_t = util.parse_time(ur_msgs)

    robotiq_force = np.array([[msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z] for msg in robotiq_msgs])
    ur_force = np.array([[msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z] for msg in ur_msgs])

    plt.figure()
    plt.grid()
    plt.plot(robotiq_t, robotiq_force[:, 0], label='fx')
    plt.plot(robotiq_t, robotiq_force[:, 1], label='fy')
    plt.plot(robotiq_t, robotiq_force[:, 2], label='fz')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Robotiq Force')

    plt.figure()
    plt.grid()
    plt.plot(ur_t, ur_force[:, 0], label='fx')
    plt.plot(ur_t, ur_force[:, 1], label='fy')
    plt.plot(ur_t, ur_force[:, 2], label='fz')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('UR Force')

    plt.show()


if __name__ == '__main__':
    main()

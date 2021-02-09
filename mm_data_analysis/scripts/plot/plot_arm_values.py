#!/usr/bin/env python2
import numpy as np
import sys
import rosbag
import mm_data_analysis.util as util
import matplotlib.pyplot as plt

import IPython


def main():
    bag = rosbag.Bag(sys.argv[1])

    joint_msgs = [msg for _, msg, _ in bag.read_messages('/ur10_joint_states')]
    t = util.parse_time(joint_msgs)

    qs = np.array([msg.position for msg in joint_msgs])
    dqs = np.array([msg.position for msg in joint_msgs])

    plt.figure()
    plt.grid()
    plt.plot(t, qs[:, 0], label='q1')
    plt.plot(t, qs[:, 1], label='q2')
    plt.plot(t, qs[:, 2], label='q3')
    plt.plot(t, qs[:, 3], label='q4')
    plt.plot(t, qs[:, 4], label='q5')
    plt.plot(t, qs[:, 5], label='q6')
    plt.xlabel('Time (s)')
    plt.ylabel('Joint angles (rad)')
    plt.legend()
    plt.title('Arm Joint Angles')
    plt.show()


if __name__ == '__main__':
    main()

#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import matplotlib.pyplot as plt
import mm_data_analysis.util as util
from mm_kinematics import KinematicModel

import IPython


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    model = KinematicModel()

    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]
    joint_msgs_traj = util.trim_to_traj(joint_msgs, pose_msgs)

    t0 = util.msg_time(pose_msgs[0])
    t = util.parse_time(joint_msgs_traj, t0=t0)

    ms = [model.manipulability(msg.position) for msg in joint_msgs_traj]

    plt.figure()
    plt.plot(t, ms)
    plt.grid()
    plt.legend()
    plt.title('Manipulability Index vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Manipulability Index')
    plt.show()


if __name__ == '__main__':
    main()

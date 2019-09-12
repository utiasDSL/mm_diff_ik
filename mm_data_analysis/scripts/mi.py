#!/usr/bin/env python2
# Analyse and plot manipulability maximization objective experimental results
from __future__ import print_function

import numpy as np
import rosbag
import matplotlib.pyplot as plt
import mm_data_analysis.util as util

import mm_kinematics.kinematics as kinematics


BAG0 = 'bags/2019-09-10/baseline/line_v0.1_2019-09-10-14-03-05.bag'

BAGS1 = [
    'bags/2019-09-12/mi/line_mi1_w1_2019-09-12-13-47-08.bag',
    'bags/2019-09-12/mi/line_mi1_w10_2019-09-12-13-49-22.bag',
    'bags/2019-09-12/mi/line_mi1_w100_2019-09-12-13-52-07.bag',
    'bags/2019-09-12/mi/line_mi1_w1000_2019-09-12-13-54-19.bag',
    'bags/2019-09-12/mi/line_mi1_w10000_2019-09-12-13-57-52.bag'
]

BAGS2 = [
    'bags/2019-09-12/mi/line_mi2_w1_2019-09-12-14-02-11.bag',
    'bags/2019-09-12/mi/line_mi2_w10_2019-09-12-14-04-07.bag',
    'bags/2019-09-12/mi/line_mi2_w100_2019-09-12-14-05-54.bag',
    'bags/2019-09-12/mi/line_mi2_w1000_2019-09-12-14-08-15.bag',
    'bags/2019-09-12/mi/line_mi2_w10000_2019-09-12-14-11-17.bag'
]

WEIGHTS = [1, 10, 100, 1000, 10000]


def calc_mi(joint_msgs, pose_msgs):
    joint_msgs = util.trim_to_traj(joint_msgs, pose_msgs)
    t = util.parse_time(joint_msgs)
    qs = np.array([msg.position for msg in joint_msgs])

    # calculate MI at each timestep
    mi = np.zeros(len(t))
    for i in xrange(len(mi)):
        mi[i] = kinematics.manipulability(qs[i,:])
    return t, mi


def parse_bag(w, name):
    bag = rosbag.Bag(name)
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]

    t0 = pose_msgs[0].header.stamp.to_sec()
    tf = pose_msgs[-1].header.stamp.to_sec()
    freq = len(pose_msgs) / (tf - t0)

    t, mi = calc_mi(joint_msgs, pose_msgs)
    print('w = {}: avg MI = {}; f = {}'.format(w, np.mean(mi), freq))
    return t, mi, freq


def main():
    t0, mi0, _ = parse_bag(0, BAG0)

    t1 = []
    mi1 = []
    f1 = []

    t2 = []
    mi2 = []
    f2 = []

    for i in xrange(len(WEIGHTS)):
        t, mi, f = parse_bag(WEIGHTS[i], BAGS1[i])
        t1.append(t)
        mi1.append(mi)
        f1.append(f)

        t, mi, f = parse_bag(WEIGHTS[i], BAGS2[i])
        t2.append(t)
        mi2.append(mi)
        f2.append(f)

    plt.figure()
    for i in xrange(len(t1)):
        plt.plot(t1[i], mi1[i], label='$\\alpha={}$'.format(WEIGHTS[i]), linewidth=3)
    plt.legend()
    plt.title('First-order Approximation')
    plt.xlabel('Time (s)')
    plt.ylabel('Manipulability Index')

    plt.figure()
    for i in xrange(len(t1)):
        plt.plot(t2[i], mi2[i], label='$\\alpha={}$'.format(WEIGHTS[i]), linewidth=3)
    plt.legend()
    plt.title('Second-order Approximation')
    plt.xlabel('Time (s)')
    plt.ylabel('Manipulability Index')

    print('avg freq (1st) = {}'.format(np.mean(f1)))
    print('avg freq (2nd) = {}'.format(np.mean(f2)))

    plt.show()


if __name__ == '__main__':
    main()

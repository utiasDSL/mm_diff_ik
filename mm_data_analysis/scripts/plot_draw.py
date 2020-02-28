#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import mm_data_analysis.util as util

import IPython


T_MAX = 60


def cutoff_idx(t):
    for i in xrange(len(t)):
        if t[i] > T_MAX:
            return i
    return -1


def parse_bag(bag):
    force_info_msgs = [msg for _, msg, _ in bag.read_messages('/force/info')]
    force_msg_times = [t.to_sec() for _, _, t in bag.read_messages('/force/info')]

    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    pose_msg_times = [t.to_sec() for _, _, t in bag.read_messages('/mm_pose_state')]

    # align pose messages with force
    aligned_msgs = util.align_lists(force_msg_times, force_info_msgs, pose_msg_times, pose_msgs)
    pose_msgs_aligned = [msg for _, msg in aligned_msgs]
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]

    t = np.array(force_msg_times) - force_msg_times[0]

    # parse positions
    pa_msgs = [msg.actual.position for msg in pose_msgs_aligned]
    pa = np.array([[msg.x, msg.y, msg.z] for msg in pa_msgs])
    pa = pa - pa[0, :]  # normalize to 0 initial position

    pd_msgs = [msg.desired.position for msg in pose_msgs_aligned]
    pd = np.array([[msg.x, msg.y, msg.z] for msg in pd_msgs])
    pd = pd - pd[0, :]  # normalize to 0 initial position

    # parse force
    f = np.array([[msg.force_world.x, msg.force_world.y, msg.force_world.z]
                  for msg in force_info_msgs])
    f = np.linalg.norm(f, axis=1)

    # start when the actual circle trajectory does
    count = 0
    start_idx = 0
    for i in xrange(1, pd.shape[0]):
        if np.abs(pd[i, 0] - pd[i-1, 0]) > 0.001:
            count += 1
        if count >= 2:
            start_idx = i
            break
    cutoff = cutoff_idx(t) + start_idx

    return t[start_idx:cutoff] - t[start_idx], pa[start_idx:cutoff], pd[start_idx:cutoff], f[start_idx:cutoff]


def main():
    bagname = util.arg_or_most_recent('2020-02-27/draw/circle_okay.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    t, pa, pd, f = parse_bag(bag)

    fig1 = plt.figure(1, figsize=(3.25, 2))
    matplotlib.rcParams.update({'font.size': 8,
                                'text.usetex': True,
                                'text.latex.preamble': ['\usepackage{bm}'],
                                'legend.fontsize': 6})

    plt.subplot(211)
    ax1 = plt.gca()
    ax1.set_xticklabels([])
    plt.plot(t, f, color='k')
    plt.ylabel('$\|\\bm{f}_e\|\mathrm{\ (N)}$')

    plt.subplot(212)
    ax2 = plt.gca()
    plt.plot(t,  pa[:, 0], label='$\mathrm{Actual}$')
    plt.plot(t,  pd[:, 0], label='$\mathrm{Desired}$', linestyle='--')
    plt.legend()
    plt.ylabel('$x\mathrm{\ (m)}$')
    plt.xlabel('$\mathrm{Time\ (s)}$')

    # bbox = ax1.get_position()
    # offset = -0.1
    # ax1.set_position([bbox.x0, bbox.y0 + offset, bbox.x1-bbox.x0, bbox.y1 - bbox.y0])

    fig2 = plt.figure(2, figsize=(1.75, 1.75))
    ax3 = plt.gca()
    plt.plot(-1*pa[:, 1], pa[:, 2], label='$\mathrm{Actual}$')
    plt.plot(-1*pd[:, 1], pd[:, 2], label='$\mathrm{Desired}$', linestyle='--')
    c1 = plt.rcParams['axes.prop_cycle'].by_key()['color'][0]
    plt.plot(pa[0, 1], pa[0, 2], 'o', color=c1, markersize=5)
    ax3.set_yticks([-0.2, 0, 0.2])

    plt.xlabel('$y\mathrm{\ (m)}$')
    plt.ylabel('$z\mathrm{\ (m)}$')
    plt.legend(loc='center')

    fig1.align_ylabels()
    fig1.tight_layout(pad=0.1)
    fig1.savefig('draw1.pdf')

    fig2.tight_layout(pad=0.1)
    fig2.savefig('draw2.pdf')
    # plt.show()


if __name__ == '__main__':
    main()

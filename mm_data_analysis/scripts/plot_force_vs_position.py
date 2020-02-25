#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import mm_data_analysis.util as util

import IPython


BAG_W0  = 'compliance/compliance_wp0.bag'
BAG_W01 = 'compliance/compliance_wp0.1.bag'
BAG_W1  = 'compliance/compliance_wp1.bag'
BAG_W10 = 'compliance/compliance_wp10.bag'

T_MAX = 60


def cutoff_idx(t):
    for i in xrange(len(t)):
        if t[i] > T_MAX:
            return i
    return -1


def parse_compliance_bag(bag):
    force_info_msgs = [msg for _, msg, _ in bag.read_messages('/force/info')]
    force_msg_times = [t.to_sec() for _, _, t in bag.read_messages('/force/info')]

    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    pose_msg_times = [t.to_sec() for _, _, t in bag.read_messages('/mm_pose_state')]

    # align pose messages with force
    aligned_msgs = util.align_lists(force_msg_times, force_info_msgs, pose_msg_times, pose_msgs)
    pose_msgs_aligned = [msg for _, msg in aligned_msgs]

    # parse time
    t = np.array(force_msg_times) - force_msg_times[0]
    cutoff = cutoff_idx(t)

    # parse force
    fs = np.array([[msg.force_world.x, msg.force_world.y, msg.force_world.z]
                   for msg in force_info_msgs])

    # parse positions
    pos_msgs = [msg.error.position for msg in pose_msgs_aligned]
    ps = np.array([[msg.x, msg.y, msg.z] for msg in pos_msgs])
    ps = ps - ps[0, :]  # normalize to 0 initial position

    # IPython.embed()

    return t[:cutoff], fs[:cutoff, :], ps[:cutoff, :]


def main():
    # bagname = util.arg_or_most_recent('*.bag')
    # print(bagname)
    bag_w0  = rosbag.Bag(BAG_W0)
    bag_w01 = rosbag.Bag(BAG_W01)
    bag_w1  = rosbag.Bag(BAG_W1)
    bag_w10 = rosbag.Bag(BAG_W10)

    t0,  f0,  p0  = parse_compliance_bag(bag_w0)
    t01, f01, p01 = parse_compliance_bag(bag_w01)
    t1,  f1,  p1  = parse_compliance_bag(bag_w1)
    t10, f10, p10 = parse_compliance_bag(bag_w10)

    fig = plt.figure(figsize=(3.25, 2.5))
    ax = plt.gca()
    matplotlib.rcParams.update({'font.size': 8,
                        'text.usetex': True,
                        'legend.fontsize': 8})
    plt.rc('text', usetex=True)

    #ax.grid()
    ax.plot(t0,  f0[:, 0],  label='$f_x$', color='k')

    ax2 = ax.twinx()

    ax2.plot(t0,  p0[:, 0],  label='$w_p=0$')
    ax2.plot(t01, p01[:, 0], label='$w_p=0.1$')
    ax2.plot(t1,  p1[:, 0],  label='$w_p=1$')
    ax2.plot(t10, p10[:, 0], label='$w_p=10$')

    ax2.set_ylabel('$\mathrm{Position\ (m)}$')
    ax2.set_ylim([-2.5, 1.0])
    ax2.set_yticks([-2.0, -1.5, -1.0, -0.5, 0.0, 0.5])

    ax.set_ylim([-16, 4])
    ax.set_xlabel('$\mathrm{Time\ (s)}$')
    ax.set_ylabel('$\mathrm{Force\ (N)}$')
    # plt.title('$\mathrm{Compliant\ Position\ Offset\ vs.\ Applied\ Force}$')
    plt.legend(loc='upper center', ncol=2)

    fig.tight_layout(pad=0.1)
    fig.savefig('compliance.pdf')
    plt.show()


if __name__ == '__main__':
    main()

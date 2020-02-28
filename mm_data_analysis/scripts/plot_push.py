#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import mm_data_analysis.util as util
import mm_kinematics.kinematics as kinematics

import IPython


T_MAX = 60


def parse_msgs(bag, topic):
    msgs = [msg for _, msg, _ in bag.read_messages(topic)]
    times = [t.to_sec() for _, _, t in bag.read_messages(topic)]
    return msgs, times


def align_msgs(t1, l1, t2, l2):
    aligned_msgs = util.align_lists(t1, l1, t2, l2)
    return [msg for _, msg in aligned_msgs]


def cutoff_idx(t):
    for i in xrange(len(t)):
        if t[i] > T_MAX:
            return i
    return -1


def parse_push_bag(bag):
    force_info_msgs, force_msg_times = parse_msgs(bag, '/force/info')
    pose_msgs, pose_msg_times = parse_msgs(bag, '/mm_pose_state')
    barrel_msgs, barrel_msg_times = parse_msgs(bag, '/vicon/Barrel/Barrel')

    # align pose messages with force
    pose_msgs_aligned = align_msgs(force_msg_times, force_info_msgs, pose_msg_times, pose_msgs)
    barrel_msgs_aligned = align_msgs(force_msg_times, force_info_msgs, barrel_msg_times, barrel_msgs)

    # parse time
    t = np.array(force_msg_times) - force_msg_times[0]
    cutoff = cutoff_idx(t)

    # parse force
    fs = np.array([[msg.force_world.x, msg.force_world.y, msg.force_world.z]
                   for msg in force_info_msgs])

    # EE positions
    pos_msgs = [msg.actual.position for msg in pose_msgs_aligned]
    ps = np.array([[msg.x, msg.y, msg.z] for msg in pos_msgs])
    p0 = ps[0, :]
    ps = ps - p0  # normalize to 0 initial position

    # v = np.zeros(ps.shape)
    # for i in xrange(len(pose_msgs_aligned)):
    #     q = np.array(pose_msgs_aligned[i].q)
    #     dq = np.array(pose_msgs_aligned[i].dq)
    #     Jp = kinematics.jacobian(q)[:3, :]
    #     v[i, :] = Jp.dot(dq)

    # barrel positions
    bps = np.array([[msg.transform.translation.x, msg.transform.translation.y,
                     msg.transform.translation.z] for msg in barrel_msgs_aligned])
    bp0 = p0
    bps = bps - bp0  # normal w.r.t. EE initial position

    for i in xrange(bps.shape[0]):
        if np.linalg.norm(bps[i, :] - bps[0, :]) > 0.01:
            print(t[i])
            move_idx = i
            break

    v = np.zeros((bps.shape[0] - 1, 3))
    for i in xrange(1, bps.shape[0]):
        dt = t[i] - t[i-1]
        dp = bps[i, :] - bps[i-1, :]
        v[i-1, :] = dp / dt

    cutoff = len(t)
    print(np.mean(v[move_idx:, 0]))

    # IPython.embed()

    return t[:cutoff], fs[:cutoff, :], ps[:cutoff, :], v[:cutoff, :], bps[:cutoff, :]


def main():
    # bagname = util.arg_or_most_recent('*.bag')
    # print(bagname)
    # bag = rosbag.Bag(bagname)
    bag = rosbag.Bag('2020-02-27/push/2020-02-27-15-21-07.bag')
    # bag = rosbag.Bag('2020-02-25/push/2020-02-25-20-54-53.bag')
    # bag = rosbag.Bag('2020-02-25/push/2020-02-25-21-14-25.bag')
    # bag = rosbag.Bag('2020-02-25/push/push_w11_5.bag')

    t, f, p, v, bp = parse_push_bag(bag)

    IPython.embed()

    fig = plt.figure(figsize=(3.25, 3))
    matplotlib.rcParams.update({'font.size': 8,
                        'text.usetex': True,
                        'text.latex.preamble': ['\usepackage{bm}'],
                        'legend.fontsize': 6})

    plt.subplot(311)
    ax = plt.gca()
    ax.set_xticklabels([])
    plt.plot(t,  f[:, 0], label='$f_x$', color='k')
    plt.ylabel('$\|\\bm{f}_e\|\mathrm{\ (N)}$', labelpad=0.1)
    # ax.set_yticks([-16, -12, -8, -4, 0])

    plt.subplot(312)
    ax2 = plt.gca()
    ax2.set_xticklabels([])

    plt.plot(t,  p[:, 1],  label='$\mathrm{EE}$')
    plt.plot(t,  bp[:, 1],  label='$\mathrm{Barrel}$')

    # ax2.add_patch(plt.Circle((20, 0), 0.2, color='r', linewidth=1, fill=False), xycoords='figure pixels')
    patch1 = plt.Circle((0.73, 0.45), 0.05, fill=False, color='r', linewidth=1,
                       linestyle='--', alpha=1, zorder=1000,
                       transform=fig.transFigure, figure=fig)
    patch2 = plt.Circle((0.55, 0.55), 0.05, fill=False, color='r', linewidth=1,
                       linestyle='--', alpha=1, zorder=1000,
                       transform=fig.transFigure, figure=fig)

    fig.patches.extend([patch1, patch2])

    plt.ylabel('$y\mathrm{\ (m)}$', labelpad=0.1)
    # plt.set_ylim([-2.5, 1.0])
    # ax2.set_yticks([-2.0, -1.5, -1.0, -0.5, 0.0])
    # plt.set_ylim([-16, 4])
    plt.legend()

    plt.subplot(313)
    plt.plot(t, p[:, 0], label='EE')
    plt.plot(t, bp[:, 0], label='Barrel')
    plt.ylabel('$x\mathrm{\ (m)}$', labelpad=0.1)
    plt.xlabel('$\mathrm{Time\ (s)}$', labelpad=0)

    # fig.subplots_adjust(hspace=0.5)
    fig.align_ylabels()
    fig.tight_layout(pad=0.1)
    fig.savefig('push.pdf')
    # plt.show()


if __name__ == '__main__':
    main()

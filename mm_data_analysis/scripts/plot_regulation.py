#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import mm_data_analysis.util as util
import tf.transformations as tfs

import IPython


T_MAX = 45


def cutoff_idx(t):
    for i in xrange(len(t)):
        if t[i] > T_MAX:
            return i
    return -1


def parse_regulation_bag(bag):
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

    quat_msgs = [msg.actual.orientation for msg in pose_msgs_aligned]
    qs = np.array([[msg.x, msg.y, msg.z, msg.w] for msg in quat_msgs])

    ne = np.zeros(ps.shape)
    for i in xrange(qs.shape[0]):
        q = qs[i, :]
        T = tfs.quaternion_matrix(q)
        R = T[:3, :3]
        z = np.array([0, 0, 1])  # with zero rotation, z is normal
        ne[i, :] = R.dot(z)

    f_norms = np.linalg.norm(fs, axis=1)
    nf = np.zeros(fs.shape)
    for i in xrange(len(f_norms)):
        if f_norms[i] > 0:
            start_idx = i
            break
        # if f_norms[i] == 0:
        #     nf[i, :] = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0])
        # else:
        #     nf[i, :] = fs[i, :] / f_norms[i]
    nf = fs / f_norms[:, None]
    f_err = 10 - f_norms

    thetas = np.arccos(np.sum(nf * ne, axis=1))

    return t[start_idx:cutoff] - t[start_idx], f_err[start_idx:cutoff], thetas[start_idx:cutoff]


def main():
    # bagname = util.arg_or_most_recent('*.bag')
    # print(bagname)

    bag_w0 = rosbag.Bag('2020-02-27-01-56-08.bag')
    bag_w10 = rosbag.Bag('2020-02-26-22-28-02.bag')
    bag_w30 = rosbag.Bag('2020-02-26-23-42-38.bag')
    bag_w100 = rosbag.Bag('2020-02-26-23-45-51.bag')

    t0, f0, thetas0 = parse_regulation_bag(bag_w0)
    t10, f10, thetas10 = parse_regulation_bag(bag_w10)
    t30, f30, thetas30 = parse_regulation_bag(bag_w30)
    t100, f100, thetas100 = parse_regulation_bag(bag_w100)

    fig = plt.figure(figsize=(3.25, 2.1))
    matplotlib.rcParams.update({'font.size': 8,
                        'text.usetex': True,
                        'legend.fontsize': 6})

    plt.subplot(211)
    ax1 = plt.gca()
    ax1.set_xticklabels([])

    plt.grid()
    plt.plot(t100, f100, label='$w_{\epsilon,f}=100$')
    plt.plot(t30, f30, label='$w_{\epsilon,f}=30$')
    plt.plot(t10, f10, label='$w_{\epsilon,f}=10$')
    plt.plot(t0, f0, label='$w_{\epsilon,f}=0$')
    plt.ylabel('$\Delta f\mathrm{\ (N)}$')

    # reverse order of legend entries, since they're added in reverse order for
    # overlap purposes
    handles, labels = ax1.get_legend_handles_labels()
    labels, handles = zip(*reversed(zip(labels, handles)))
    plt.legend(handles, labels, facecolor='white', framealpha=1)

    plt.subplot(212)
    ax2 = plt.gca()
    plt.grid()
    plt.plot(t100, thetas100, label='100')
    plt.plot(t30, thetas30, label='30')
    plt.plot(t10, thetas10, label='10')
    # plt.plot(t0, thetas0, label='0')
    plt.ylabel('$\\theta\mathrm{\ (rad)}$')
    plt.xlabel('$\mathrm{Time\ (s)}$')
    ax2.set_yticks([0, 0.4, 0.8])

    fig.align_ylabels()
    fig.tight_layout(pad=0.1)
    fig.savefig('regulation.pdf')

    plt.show()



if __name__ == '__main__':
    main()

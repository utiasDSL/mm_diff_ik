#!/usr/bin/env python2
import sys
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import mm_kinematics.kinematics as kinematics
import mm_data_analysis.util as util
import tf.transformations as tfs

import IPython


# TODO second point may not be correct
OBS_X = [0, 1]
OBS_Y = [-0.5, 0.5]


def closest_dist_to_obs(ps, o):
    closest_idx = 0
    closest_dist = np.linalg.norm(ps[0,:2] - o)
    for i in xrange(ps.shape[0]):
        p = ps[i, :2]
        dist = np.linalg.norm(p - o)
        if dist < closest_dist:
            closest_dist = dist
            closest_idx = i
    return ps[closest_idx, :2]


def parse_obstacles(obs_msgs):
    o1s = []
    o2s = []
    ts = []
    for msg in obs_msgs:
        if len(msg.obstacles) > 0:
            o1s.append(msg.obstacles[0].centre)
            o2s.append(msg.obstacles[1].centre)
            ts.append(msg.header.stamp.to_sec())
    return ts, util.vec3_msg_to_np(o1s), util.vec3_msg_to_np(o2s)


def main():
    bag = rosbag.Bag(sys.argv[1])
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]
    obs_msgs = [msg for _, msg, _ in bag.read_messages('/obstacles')]

    joint_msgs = util.trim_to_traj(joint_msgs, pose_msgs)
    t_b = util.parse_time(joint_msgs, normalize_time=False)

    obs_msgs = util.trim_to_traj(obs_msgs, pose_msgs)

    qs = [msg.position for msg in joint_msgs]

    t_o, o1s, o2s = parse_obstacles(obs_msgs)

    IPython.embed()
    return

    # Normalize times
    t_o -= t_b[0]
    t_b -= t_b[0]

    pos_bs = []  # base positions
    pos_es = []  # EE positions

    for q in qs:
        Ts = kinematics.forward_chain(q)
        w_T_b = Ts[3]
        w_T_e = Ts[-1]
        w_p_b = tfs.translation_from_matrix(w_T_b)
        w_p_e = tfs.translation_from_matrix(w_T_e)
        pos_bs.append(w_p_b)
        pos_es.append(w_p_e)

    pos_bs = np.array(pos_bs)
    pos_es = np.array(pos_es)

    fig = plt.figure(figsize=(3.25, 2))
    plt.rcParams.update({'font.size': 8,
                         'text.usetex': True})

    plt.subplot(211)
    plt.plot(t_b, pos_bs[:,0], label='Base', linewidth=2, color='r')

    plt.plot(t_o, o1s[:,0], color='k', linestyle='dashed')
    plt.plot(t_o, o2s[:,0], color='k', linestyle='dashed')

    plt.ylabel('$x$ (m)')

    plt.legend(labelspacing=0.1, borderpad=0.3, loc=0)
    # plt.xlim([-1.1, 3.3])
    # plt.ylim([-1, 1])
    #
    # plt.yticks([-1, 0, 1])
    plt.grid()

    plt.subplot(212)
    plt.plot(t_b, pos_bs[:,1], linewidth=2, color='r')

    plt.xlabel('Time (s)')
    plt.ylabel('$y$ (m)')

    fig.tight_layout(pad=0.1)
    fig.savefig('dyn_obstacles.pdf')

    plt.show()


if __name__ == '__main__':
    main()

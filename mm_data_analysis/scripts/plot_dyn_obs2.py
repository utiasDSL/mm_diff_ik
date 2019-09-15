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


def closest_pair(pairs):
    idx = 0
    least = np.linalg.norm(pairs[0][0] - pairs[0][1])
    for i, pair in enumerate(pairs):
        d = np.linalg.norm(pair[0] - pair[1])
        if d < least:
            least = d
            idx = i
    return pairs[idx]


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

    # only care about messages that fall during the trajectory
    joint_msgs = util.trim_to_traj(joint_msgs, pose_msgs)
    obs_msgs = util.trim_to_traj(obs_msgs, pose_msgs)

    t_b = util.parse_time(joint_msgs, normalize_time=False)

    t_o, o1s, o2s = parse_obstacles(obs_msgs)

    # Normalize times
    t_o -= t_b[0]
    t_b -= t_b[0]


    # t_o, o1s, o2s = parse_obstacles(obs_msgs)

    qs = [msg.position for msg in joint_msgs]

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

    # pair obstacles and joint states so we can discover closest obstacle
    aligned1 = util.align_lists(t_o, o1s, t_b, pos_bs)
    aligned2 = util.align_lists(t_o, o2s, t_b, pos_bs)

    pair1 = closest_pair(aligned1)
    pair2 = closest_pair(aligned2)

    # IPython.embed()
    # return

    # o1 = np.array([OBS_X[0], OBS_Y[0]])
    # o2 = np.array([OBS_X[1], OBS_Y[1]])
    # p1 = closest_dist_to_obs(pos_bs, o1)
    # p2 = closest_dist_to_obs(pos_bs, o2)

    o1 = pair1[0][:2]
    o2 = pair2[0][:2]
    p1 = pair1[1][:2]
    p2 = pair2[1][:2]

    fig = plt.figure(figsize=(3.25, 2))
    plt.rcParams.update({'font.size': 8,
                         'text.usetex': True})
    ax = plt.gca()

    # Obstacles
    ax.add_patch(plt.Circle(o1, 0.1, color='k'))
    ax.add_patch(plt.Circle(o2, 0.1, color='k'))
    ax.add_patch(plt.Circle(o1, 0.2, color='k', linewidth=1, fill=False))
    ax.add_patch(plt.Circle(o2, 0.2, color='k', linewidth=1, fill=False))

    # Start positions of base and EE
    ax.add_patch(plt.Circle((pos_bs[0, 0], pos_bs[0, 1]), 0.05, color='r'))
    ax.add_patch(plt.Circle((pos_es[0, 0], pos_es[0, 1]), 0.05, color='b'))

    # Closest points of base to each obstacle
    ax.add_patch(plt.Circle(p1, 0.5, color='r', linewidth=1,
                             linestyle='dashed', fill=False))
    ax.add_patch(plt.Circle(p2, 0.5, color='r', linewidth=1,
                             linestyle='dashed', fill=False))

    plt.axis('scaled')
    plt.plot(pos_bs[:,0], pos_bs[:,1], label='Base', linewidth=2, color='r')
    plt.plot(pos_es[:,0], pos_es[:,1], label='End effector', linewidth=2, color='b')

    plt.xlabel('$x$ (m)')
    plt.ylabel('$y$ (m)')

    plt.legend(labelspacing=0.1, borderpad=0.3, loc=0)
    plt.xlim([-1.1, 3.3])
    plt.ylim([-1, 1])

    plt.yticks([-1, 0, 1])
    plt.grid()

    fig.tight_layout(pad=0.1)
    fig.savefig('obstacles.pdf')

    plt.show()


if __name__ == '__main__':
    main()

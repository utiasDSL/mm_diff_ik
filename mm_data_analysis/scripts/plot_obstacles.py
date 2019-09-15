#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import mm_kinematics.kinematics as kinematics
import tf.transformations as tfs

import IPython


BAG = 'bags/2019-09-12/obstacles/obs_sim_2019-09-12-14-40-06.bag'


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


def main():
    bag = rosbag.Bag(BAG)
    joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]
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

    o1 = np.array([OBS_X[0], OBS_Y[0]])
    o2 = np.array([OBS_X[1], OBS_Y[1]])
    p1 = closest_dist_to_obs(pos_bs, o1)
    p2 = closest_dist_to_obs(pos_bs, o2)

    fig = plt.figure(figsize=(3.25, 2))
    plt.rcParams.update({'font.size': 8,
                         'text.usetex': True})
    ax = plt.gca()

    # Obstacles
    ax.add_patch(plt.Circle((OBS_X[0], OBS_Y[0]), 0.1, color='k'))
    ax.add_patch(plt.Circle((OBS_X[1], OBS_Y[1]), 0.1, color='k'))

    ax.add_patch(plt.Circle((OBS_X[0], OBS_Y[0]), 0.2, color='k', linewidth=1, fill=False))
    ax.add_patch(plt.Circle((OBS_X[1], OBS_Y[1]), 0.2, color='k', linewidth=1, fill=False))

    # Start positions of base and EE
    ax.add_patch(plt.Circle((pos_bs[0, 0], pos_bs[0, 1]), 0.05, color='r'))
    ax.add_patch(plt.Circle((pos_es[0, 0], pos_es[0, 1]), 0.05, color='b'))

    # Closest points
    ax.add_patch(plt.Circle((p1[0], p1[1]), 0.5, color='r', linewidth=1,
                             linestyle='dashed', fill=False))
    ax.add_patch(plt.Circle((p2[0], p2[1]), 0.5, color='r', linewidth=1,
                             linestyle='dashed', fill=False))


    plt.axis('scaled')
    plt.plot(OBS_X, OBS_Y, 'o', c='k')
    plt.plot(pos_bs[:,0], pos_bs[:,1], label='Base', linewidth=2, color='r')
    plt.plot(pos_es[:,0], pos_es[:,1], label='End effector', linewidth=2, color='b')

    plt.plot(p1[0], p1[1], markersize=10, marker='+', color='k')
    plt.plot(p2[0], p2[1], markersize=10, marker='+', color='k')

    plt.xlabel('x (m)')
    plt.ylabel('y (m)')

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

#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import mm_data_analysis.util as util
import mm_kinematics.kinematics as kinematics
import tf.transformations as tfs

import IPython


LINE_BAG = 'bags/2019-09-15/slalom/line/line_v0.4_slalom_2019-09-15-14-10-43.bag'
FORCE_BAG = 'bags/2019-09-15/slalom/force/slalom_force2_2019-09-15-14-27-16.bag'


# Obstacle locations
OBS_X = [-1, 0, 1, 2]
OBS_Y = [-0.5, 0.5, -0.5, 0.5]


def plot_line(fig):
    bag = rosbag.Bag(LINE_BAG)
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]
    joint_msgs = util.trim_to_traj(joint_msgs, pose_msgs)
    qs = [msg.position for msg in joint_msgs]

    # d = pose_msgs[-1].desired.position.x - pose_msgs[0].desired.position.x
    # print('EE distance = {}'.format(d))

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

    ax = plt.gca()
    ax.set_title('Experiment 1: Trajectory Tracking', fontsize=8)

    # Obstacles
    for i in range(len(OBS_X)):
        ax.add_patch(plt.Circle((OBS_X[i], OBS_Y[i]), 0.1, color='k'))
        ax.add_patch(plt.Circle((OBS_X[i], OBS_Y[i]), 0.2, color='k', linewidth=1, fill=False))

    # Start positions of base and EE
    ax.add_patch(plt.Circle((pos_bs[0, 0], pos_bs[0, 1]), 0.05, color='r'))
    ax.add_patch(plt.Circle((pos_es[0, 0], pos_es[0, 1]), 0.05, color='b'))

    plt.axis('scaled')
    plt.plot(pos_bs[:,0], pos_bs[:,1], label='Base', linewidth=2, color='r')
    plt.plot(pos_es[:,0], pos_es[:,1], label='End effector', linewidth=2, color='b')

    ax.set_xticklabels([])
    plt.ylabel('y (m)')

    plt.xlim([-2.1, 4.3])
    plt.ylim([-1, 1])

    plt.yticks([-1, 0, 1])
    plt.grid()


def plot_force(fig):
    bag = rosbag.Bag(FORCE_BAG)
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

    ax = plt.gca()
    ax.set_title('Experiment 2: Force Adaption', fontsize=8)

    # Obstacles
    for i in range(len(OBS_X)):
        ax.add_patch(plt.Circle((OBS_X[i], OBS_Y[i]), 0.1, color='k'))
        ax.add_patch(plt.Circle((OBS_X[i], OBS_Y[i]), 0.2, color='k', linewidth=1, fill=False))

    # Start positions of base and EE
    ax.add_patch(plt.Circle((pos_bs[0, 0], pos_bs[0, 1]), 0.05, color='r'))
    ax.add_patch(plt.Circle((pos_es[0, 0], pos_es[0, 1]), 0.05, color='b'))

    plt.axis('scaled')
    plt.plot(pos_bs[:,0], pos_bs[:,1], label='Base', linewidth=2, color='r')
    plt.plot(pos_es[:,0], pos_es[:,1], label='End effector', linewidth=2, color='b')

    plt.legend(labelspacing=0.1, borderpad=0.3, loc=4)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')

    plt.xlim([-2.1, 4.3])
    plt.ylim([-1, 1])

    ax.set_xticklabels(['', '$0$', '$1$', '$2$', '$3$', '$4$', '$5$', '$6$'])

    plt.yticks([-1, 0, 1])
    plt.grid()


def main():
    fig = plt.figure(figsize=(3.25, 2.5))
    plt.rcParams.update({'font.size': 8,
                         'text.usetex': True,
                         'legend.fontsize': 8})

    plt.subplot(211)
    plot_line(fig)

    plt.subplot(212)
    plot_force(fig)

    fig.tight_layout(pad=0.1)
    fig.savefig('slalom.pdf')
    plt.show()


if __name__ == '__main__':
    main()

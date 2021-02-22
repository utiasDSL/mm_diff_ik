#!/usr/bin/env python2
from __future__ import print_function, division

import rosbag
import numpy as np
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt
from mm_kinematics import KinematicModel
import mm_data_analysis.util as util

import IPython


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    model = KinematicModel()

    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    mm_joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]
    mm_joint_msgs_traj = util.trim_to_traj(mm_joint_msgs, pose_msgs)
    t0 = util.parse_t0(mm_joint_msgs_traj)
    t = util.parse_time(mm_joint_msgs_traj)
    qs = np.array([msg.position for msg in mm_joint_msgs_traj])
    Ts = [model.calc_T_w_tool(msg.position) for msg in mm_joint_msgs_traj]

    barrel_msgs = [msg for _, msg, _ in bag.read_messages('/vicon/Barrel/Barrel')]
    pos = [util.vec3_to_np(msg.transform.translation) for msg in barrel_msgs]
    tb = util.parse_time(barrel_msgs, t0=t0)
    pos = np.array(util.linear_interpolate_list(t, tb, pos))
    pos = pos[:, :2]  # only position

    pes = np.array([T[:2, 3] for T in Ts])  # EE positions
    pbs = qs[:, :2]  # base positions

    # normalize so base starts at x = 0
    p0 = pes[0, :]
    pbs = pbs - p0
    pes = pes - p0
    pos = pos - p0

    fig = plt.figure(figsize=(3.25, 1.5))
    plt.rcParams.update({'font.size': 8,
                         'text.usetex': True,
                         'legend.fontsize': 8})

    ax = plt.gca()
    # line1, = plt.plot(pbs[:, 0], pbs[:, 1], zorder=3, label=r"$\mathrm{Base}$")
    plt.plot(pes[:, 0], pes[:, 1], zorder=4, label=r"$\mathrm{EE}$")
    plt.plot(pos[:, 0], pos[:, 1], zorder=4, label=r"$\mathrm{Barrel}$")
    plt.ylim([-0.5, 0.5])
    # ax.set_aspect('equal')
    plt.grid()
    plt.legend()
    plt.xlabel(r'$x\ \mathrm{(m)}$')
    plt.ylabel(r'$y\ \mathrm{(m)}$')

    # Annotate perturbations
    ax.add_patch(plt.Arrow(0.96, -0.15, 0, 0.12, width=0.1, zorder=4, color="r"))
    ax.add_patch(plt.Arrow(1.65, 0.37, 0, -0.12, width=0.1, zorder=4, color="r"))

    # # draw obstacles
    # for po in pos:
    #     ax.add_patch(plt.Circle(po, 0.05, zorder=3, color='k'))
    #
    # # color = line1.get_color() + "22"
    # color = "#71c2fc33"
    #
    # # draw swept shapes around the base
    # for pb in pbs[::50]:
    #     ax.add_patch(plt.Circle(pb, 0.5, color=color, zorder=1, fill=True))
    #
    # for i in xrange(0, len(detections), 20):
    #     for detection in detections[i]:
    #         po = detection - p0
    #         plt.plot([pbs[i, 0], po[0]], [pbs[i, 1], po[1]], zorder=2, color='#88888888')

    plt.show()


if __name__ == '__main__':
    main()

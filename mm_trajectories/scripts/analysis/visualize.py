#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import mm_trajectories.trajectory as trajectory


DURATION = 30
DT = 0.1


def plot_traj(traj, ax, label):
    waypoints = trajectory.create_waypoints(traj, DURATION, DT)

    xs = [wp.pose.position.x for wp in waypoints]
    ys = [wp.pose.position.y for wp in waypoints]
    zs = [wp.pose.position.z for wp in waypoints]

    ax.plot(xs, ys, zs=zs, linewidth=2, label=label)


def main():
    quat0 = np.array([0, 0, 0, 1])

    line = trajectory.LineTrajectory(np.zeros(3), quat0, DURATION)
    sine = trajectory.SineXYTrajectory(np.zeros(3), quat0, DURATION)
    figure8 = trajectory.CircleTrajectory(np.zeros(3), quat0, DURATION)
    square = trajectory.SquareTrajectory(np.zeros(3), quat0, DURATION)
    spiral = trajectory.SpiralTrajectory(np.zeros(3), quat0, DURATION)

    fig = plt.figure(figsize=(3.25, 2.5))
    plt.rcParams.update({'font.size': 8,
                         'text.usetex': True,
                         'text.latex.unicode': True})
    ax = fig.add_subplot(111, projection='3d')

    ax.zaxis.labelpad = 40

    plot_traj(line, ax, 'Line')
    plot_traj(sine, ax, 'Sine')
    plot_traj(figure8, ax, 'Figure 8')
    plot_traj(square, ax, 'Square')
    plot_traj(spiral, ax, 'Spiral')

    ax.set_xlim([-0, 3])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([-0.5, 0.5])

    ax.set_xticks([0, 1, 2, 3])
    ax.set_yticks([-1, 0, 1])
    ax.set_zticks([-0.5, 0, 0.5])

    ax.zaxis.set_tick_params(pad=50)

    # latex and matplotlib do a bad job of locating this axis label, so do it
    # manually
    ax.set_zticklabels(['', '$0$', '$0.5$'])
    ax.annotate('$-0.5$', (12, 55), xycoords='figure pixels')

    # manually place axis labels
    ax.annotate('x (m)', (170, 15), xycoords='figure pixels')
    ax.annotate('y (m)', (40, 15), xycoords='figure pixels')
    ax.annotate('z (m)', (4, 105), xycoords='figure pixels')

    # set background colors to white
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))

    ax.xaxis.set_rotate_label(False)
    ax.yaxis.set_rotate_label(False)
    ax.zaxis.set_rotate_label(False)

    ax.view_init(elev=37, azim=-135)

    ax.legend(labelspacing=0.1, borderpad=0.2, bbox_to_anchor=(0.5, 1))

    fig.tight_layout(pad=1)
    fig.savefig('trajectories.pdf')

    # plt.show()


if __name__ == '__main__':
    main()

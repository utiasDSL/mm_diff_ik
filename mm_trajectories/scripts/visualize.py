#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
from mpl_toolkits.mplot3d import Axes3D

import mm_trajectories.trajectory as trajectory

import IPython


DURATION = 30
DT = 0.1


def plot_traj(traj, ax):
    waypoints = trajectory.create_waypoints(traj, DURATION, DT)

    xs = [wp.pose.position.x for wp in waypoints]
    ys = [wp.pose.position.y for wp in waypoints]
    zs = [wp.pose.position.z for wp in waypoints]

    ax.plot(xs, ys, zs=zs, linewidth=3)


def main():
    quat0 = np.array([0, 0, 0, 1])

    line = trajectory.LineTrajectory(np.zeros(3), quat0, DURATION)
    sine = trajectory.SineXYTrajectory(np.zeros(3), quat0, DURATION)
    figure8 = trajectory.CircleTrajectory(np.zeros(3), quat0, DURATION)
    square = trajectory.SquareTrajectory(np.zeros(3), quat0, DURATION)
    spiral = trajectory.SpiralTrajectory(np.zeros(3), quat0, DURATION)

    # rc('text', usetex=True)

    fig = plt.figure(dpi=40)
    ax = fig.add_subplot(111, projection='3d')

    ax.zaxis.labelpad = 40

    plot_traj(line, ax)
    plot_traj(sine, ax)
    plot_traj(figure8, ax)
    plot_traj(square, ax)
    plot_traj(spiral, ax)

    # ax.set_title('End Effector Trajectories')

    ax.set_xlim([-0, 3])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([-0.5, 0.5])

    ax.set_xticks([0, 1, 2, 3])
    ax.set_yticks([-1, 0, 1])
    ax.set_zticks([-0.5, 0, 0.5])

    ax.set_xlabel('\nx [m]', fontweight='bold')
    ax.set_ylabel('\ny [m]', fontweight='bold')
    ax.set_zlabel('\nz [m]', fontweight='bold', linespacing=7)

    # set background colors to white
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))

    ax.xaxis.set_rotate_label(False)
    ax.yaxis.set_rotate_label(False)
    ax.zaxis.set_rotate_label(False)

    ax.view_init(elev=37, azim=-135)

    ax.w_xaxis.set_label_coords(0, 0)

    fig.savefig('trajectories.pdf', bbox_inches='tight', pad_inches=0.2)

    plt.show()


if __name__ == '__main__':
    main()

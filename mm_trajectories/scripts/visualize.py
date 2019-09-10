#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
from mpl_toolkits.mplot3d import Axes3D

import mm_trajectories.trajectory as trajectory

import IPython


DURATION = 30
DT = 0.1


def plot_traj(traj, ax, label):
    waypoints = trajectory.create_waypoints(traj, DURATION, DT)

    xs = [wp.pose.position.x for wp in waypoints]
    ys = [wp.pose.position.y for wp in waypoints]
    zs = [wp.pose.position.z for wp in waypoints]

    ax.plot(xs, ys, zs=zs, linewidth=3, label=label)


def main():
    quat0 = np.array([0, 0, 0, 1])

    line = trajectory.LineTrajectory(np.zeros(3), quat0, DURATION)
    sine = trajectory.SineXYTrajectory(np.zeros(3), quat0, DURATION)
    figure8 = trajectory.CircleTrajectory(np.zeros(3), quat0, DURATION)
    square = trajectory.SquareTrajectory(np.zeros(3), quat0, DURATION)
    spiral = trajectory.SpiralTrajectory(np.zeros(3), quat0, DURATION)

    # rc('text', usetex=True)

    font_size = 10
    params = {
       'axes.labelsize': font_size,
       'font.size': font_size,
       'legend.fontsize': font_size,
       'xtick.labelsize': font_size,
       'ytick.labelsize': font_size,
       'figure.figsize': [6, 4]  # Set aspect ratio.
    }
    plt.rcParams.update(params)

    fig = plt.figure(dpi=40)
    ax = fig.add_subplot(111, projection='3d')

    ax.zaxis.labelpad = 40

    plot_traj(line, ax, 'Line')
    plot_traj(sine, ax, 'Sine')
    plot_traj(figure8, ax, 'Figure 8')
    plot_traj(square, ax, 'Square')
    plot_traj(spiral, ax, 'Spiral')

    # ax.set_title('End Effector Trajectories')

    ax.set_xlim([-0, 3])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([-0.5, 0.5])

    ax.set_xticks([0, 1, 2, 3])
    ax.set_yticks([-1, 0, 1])
    ax.set_zticks([-0.5, 0, 0.5])

    ax.set_xlabel('\nx [m]', fontweight='normal')
    ax.set_ylabel('\ny [m]', fontweight='normal')
    ax.set_zlabel('\nz [m]', fontweight='normal', linespacing=7)

    # set background colors to white
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))

    ax.xaxis.set_rotate_label(False)
    ax.yaxis.set_rotate_label(False)
    ax.zaxis.set_rotate_label(False)

    ax.view_init(elev=37, azim=-135)

    # ax.w_xaxis.set_label_coords(0, 0)
    ax.legend()

    fig.savefig('trajectories.pdf', bbox_inches='tight', pad_inches=0.2)

    plt.show()


if __name__ == '__main__':
    main()

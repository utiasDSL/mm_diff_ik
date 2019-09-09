#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
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

    ax.plot(xs, ys, zs=zs)


def main():
    quat0 = np.array([0, 0, 0, 1])

    line = trajectory.LineTrajectory(np.zeros(3), quat0, DURATION)
    sine = trajectory.SineXYTrajectory(np.zeros(3), quat0, DURATION)
    figure8 = trajectory.CircleTrajectory(np.zeros(3), quat0, DURATION)
    square = trajectory.SquareTrajectory(np.zeros(3), quat0, DURATION)
    spiral = trajectory.SpiralTrajectory(np.zeros(3), quat0, DURATION)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    plot_traj(line, ax)
    plot_traj(sine, ax)
    plot_traj(figure8, ax)
    plot_traj(square, ax)
    plot_traj(spiral, ax)

    ax.set_xlim([-0.1, 3.1])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([-0.5, 0.5])

    plt.show()


if __name__ == '__main__':
    main()

#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from mm_trajectories import timescaling, path, util


DURATION = 30
DT = 0.1


def plot_traj_from_waypoints(waypoints, ax, label):
    xs = [wp.pose.position.x for wp in waypoints]
    ys = [wp.pose.position.y for wp in waypoints]
    zs = [wp.pose.position.z for wp in waypoints]

    ax.plot(xs, ys, zs=zs, linewidth=2, label=label)


def plot_traj(traj, ax, label):
    waypoints = util.create_waypoints(traj, DURATION, DT)
    plot_traj_from_waypoints(waypoints, ax, label)


def make_square_waypoints(p0, quat0, s, duration):
    # s is sidelength of the square
    r = 0.5 * s
    points = p0 + np.array([[0,  0, 0],
                            [0, -r, 0],
                            [s, -r, 0],
                            [s,  r, 0],
                            [0,  r, 0],
                            [0,  0, 0]])
    durations = duration / np.array([8.0, 4.0, 4.0, 4.0, 8.0])

    waypoints = []
    for i in xrange(5):
        scaling = timescaling.QuinticTimeScaling(durations[i])
        traj = path.PointToPoint(points[i, :], points[i+1, :], quat0, scaling)
        waypoints.extend(util.create_waypoints(traj, durations[i], DT))
    return waypoints


def main():
    p0 = np.zeros(3)
    quat0 = np.array([0, 0, 0, 1])
    scaling = timescaling.QuinticTimeScaling(DURATION)

    line = path.PointToPoint(p0, p0 + [3, 0, 0], quat0, scaling)
    sine = path.SineXY(p0, quat0, 3.0, 1.0, 1.0, scaling)
    figure8 = path.Figure8(p0, quat0, 0.25, scaling)
    square_waypoints = make_square_waypoints(p0, quat0, 1.0, DURATION)
    spiral = path.Spiral(p0, quat0, 0.5, 2.0, 3.0, scaling)
    ellipse = path.Ellipse(p0, quat0, 0.2, 0.4, scaling)

    fig = plt.figure(figsize=(3.25, 2.3))
    plt.rcParams.update({'font.size': 8,
                         'text.usetex': True,
                         'text.latex.unicode': True,
                         'legend.fontsize': 8})
    plt.rcParams['xtick.major.pad'] = 0
    plt.rcParams['ytick.major.pad'] = 0
    ax = fig.add_subplot(111, projection='3d')

    ax.zaxis.labelpad = 40

    plot_traj(line, ax, r'$\mathrm{Line}$')
    plot_traj(sine, ax, r'$\mathrm{Sine}$')
    plot_traj(figure8, ax, r'$\mathrm{Figure\ 8}$')
    plot_traj_from_waypoints(square_waypoints, ax, r'$\mathrm{Square}$')
    plot_traj(ellipse, ax, r'$\mathrm{Ellipse}$')
    plot_traj(spiral, ax, r'$\mathrm{Spiral}$')

    ax.set_xlim([-0, 3])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([-0.5, 0.5])

    ax.set_xticks([0, 1, 2, 3])
    ax.set_yticks([-1, 0, 1])
    ax.set_zticks([-0.5, 0, 0.5])

    # ax.zaxis.set_tick_params(pad=50)

    # latex and matplotlib do a bad job of locating this axis label, so do it
    # manually
    # ax.set_zticklabels(['', '$0$', '$0.5$'])
    # ax.annotate('$-0.5$', (12, 55), xycoords='figure pixels')

    # manually place axis labels
    # ax.annotate('x (m)', (170, 15), xycoords='figure pixels')
    # ax.annotate('y (m)', (40, 15), xycoords='figure pixels')
    ax.annotate(r'$z\mathrm{\ (m)}$', (4, 130), xycoords='figure pixels')
    ax.set_xlabel(r'$x\mathrm{\ (m)}$')
    ax.set_ylabel(r'$y\mathrm{\ (m)}$')
    # ax.set_zlabel('$z\mathrm{\ (m)}$')

    # set background colors to white
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))

    # don't rotate labels
    ax.xaxis.set_rotate_label(False)
    ax.yaxis.set_rotate_label(False)
    ax.zaxis.set_rotate_label(False)

    # fix the 3D view angle
    ax.view_init(elev=37, azim=-135)

    ax.legend(labelspacing=0.2, ncol=2)

    fig.tight_layout(pad=1)
    fig.savefig('trajectories.pdf')

    plt.show()


if __name__ == '__main__':
    main()

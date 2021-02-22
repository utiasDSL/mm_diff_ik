#!/usr/bin/env python2
"""This script calculates the max speed of each trajectory."""
import numpy as np

from mm_trajectories import timescaling, path, util


DURATION = 30
DT = 0.1


def calc_max_vel_from_waypoints(waypoints):
    v = np.zeros(len(waypoints))
    for i, wp in enumerate(waypoints):
        vx = wp.velocity.linear.x
        vy = wp.velocity.linear.y
        vz = wp.velocity.linear.z

        v[i] = np.linalg.norm([vx, vy, vz])
    return np.max(v)


def calc_max_vel(traj):
    waypoints = util.create_waypoints(traj, DURATION, DT)
    return calc_max_vel_from_waypoints(waypoints)


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

    print('Max Line vel     = {}'.format(calc_max_vel(line)))
    print('Max Sine vel     = {}'.format(calc_max_vel(sine)))
    print('Max Figure 8 vel = {}'.format(calc_max_vel(figure8)))
    print('Max Square vel   = {}'.format(calc_max_vel_from_waypoints(square_waypoints)))
    print('Max Ellipse vel  = {}'.format(calc_max_vel(ellipse)))
    print('Max Spiral vel   = {}'.format(calc_max_vel(spiral)))


if __name__ == '__main__':
    main()

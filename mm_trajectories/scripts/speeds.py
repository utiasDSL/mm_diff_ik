#!/usr/bin/env python2
# Calculate max speed of each trajectory.
import numpy as np

import mm_trajectories.trajectory as trajectory


DURATION = 30
DT = 0.1


def calc_max_vel(traj):
    waypoints = trajectory.create_waypoints(traj, DURATION, DT)

    v = np.zeros(len(waypoints))
    for i, wp in enumerate(waypoints):
        vx = wp.velocity.linear.x
        vy = wp.velocity.linear.y
        vz = wp.velocity.linear.z

        v[i] = np.linalg.norm([vx, vy, vz])
    return np.max(v)


def main():
    quat0 = np.array([0, 0, 0, 1])

    line = trajectory.LineTrajectory(np.zeros(3), quat0, DURATION)
    sine = trajectory.SineXYTrajectory(np.zeros(3), quat0, DURATION)
    figure8 = trajectory.CircleTrajectory(np.zeros(3), quat0, DURATION)
    square = trajectory.SquareTrajectory(np.zeros(3), quat0, DURATION)
    spiral = trajectory.SpiralTrajectory(np.zeros(3), quat0, DURATION)

    print('Max Line vel = {}'.format(calc_max_vel(line)))
    print('Max Sine vel = {}'.format(calc_max_vel(sine)))
    print('Max Figure 8 vel = {}'.format(calc_max_vel(figure8)))
    print('Max Square vel = {}'.format(calc_max_vel(square)))
    print('Max Spiral vel = {}'.format(calc_max_vel(spiral)))


if __name__ == '__main__':
    main()

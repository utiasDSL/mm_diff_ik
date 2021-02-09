#!/usr/bin/env python2

from __future__ import print_function

import numpy as np
import rospy
import sys

from mm_trajectories import timescaling, path, util


DT = 0.1
S = 1.0  # side length
R = 0.5 * S


def main():
    rospy.init_node('trajectory_generator')

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30

    # wait until current pose is received
    p0, quat0 = util.wait_for_initial_pose(DT)

    points = p0 + np.array([[0,  0, 0],
                            [0, -R, 0],
                            [S, -R, 0],
                            [S,  R, 0],
                            [0,  R, 0],
                            [0,  0, 0]])
    durations = duration / np.array([8.0, 4.0, 4.0, 4.0, 8.0])

    waypoints = []
    for i in xrange(5):
        scaling = timescaling.QuinticTimeScaling(durations[i])
        traj = path.PointToPoint(points[i, :], points[i+1, :], quat0, scaling)
        waypoints.extend(util.create_waypoints(traj, durations[i], DT))

    util.publish(waypoints, DT)

    print('Launched square trajectory with duration of {} seconds.'.format(duration))


if __name__ == '__main__':
    main()

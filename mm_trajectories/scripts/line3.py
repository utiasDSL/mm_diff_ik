#!/usr/bin/env python2

from __future__ import print_function

import numpy as np
import rospy
import sys

from mm_trajectories import timescaling, path, util


DT = 0.1
OFFSET = np.array([0, 0.5, 0])


def main():
    rospy.init_node('trajectory_generator')

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30
    # duration = 30

    # wait until current pose is received
    p0, quat0 = util.wait_for_initial_pose(DT)

    p1 = p0 + OFFSET

    scaling = timescaling.QuinticTimeScaling(duration)
    traj1 = path.PointToPoint(p0, p1, quat0, scaling)
    traj2 = path.PointToPoint(p1, p0, quat0, scaling)
    traj3 = path.PointToPoint(p0, p1, quat0, scaling)

    waypoints = []
    for traj in [traj1, traj2, traj3]:
        waypoints.extend(util.create_waypoints(traj, duration, DT))

    util.publish(waypoints, DT)

    print('Launched line3 trajectory with duration of {} x3 seconds.'.format(duration))
    # print('v = {}'.format(scaling.v))
    # print('a = {}'.format(scaling.a))


if __name__ == '__main__':
    main()

#!/usr/bin/env python2

from __future__ import print_function

import rospy
import sys

from mm_trajectories import timescaling, path, util


DT = 0.1
RADIUS = 0.25


def main():
    rospy.init_node('trajectory_generator')

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30

    # wait until current pose is received
    p0, quat0 = util.wait_for_initial_pose(DT)

    scaling = timescaling.QuinticTimeScaling(duration)
    traj = path.Figure8(p0, quat0, RADIUS, scaling)
    waypoints = util.create_waypoints(traj, duration, DT)

    util.publish(waypoints, DT)

    print('Launched Figure 8 trajectory with duration of {} seconds.'.format(duration))


if __name__ == '__main__':
    main()


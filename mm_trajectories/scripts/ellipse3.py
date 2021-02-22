#!/usr/bin/env python2
"""Run ellipse trajectory three times."""

from __future__ import print_function

import rospy
import sys

from mm_trajectories import timescaling, path, util


DT = 0.1
RX = 0.2
RY = 0.4


def main():
    rospy.init_node("trajectory_generator")

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30
    # duration = 60

    # wait until current pose is received
    p0, quat0 = util.wait_for_initial_pose(DT)

    scaling = timescaling.QuinticTimeScaling(duration)
    traj = path.Ellipse(p0, quat0, RX, RY, scaling)
    waypoints = util.create_waypoints(traj, duration, DT)

    # TODO relies on the fact that dt is part of trajectory rather than each
    # waypoint
    util.publish(waypoints + waypoints + waypoints, DT)

    print("Launched ellipse trajectory with duration of {} seconds.".format(duration))


if __name__ == "__main__":
    main()

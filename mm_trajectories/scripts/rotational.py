#!/usr/bin/env python2

from __future__ import print_function

import rospy
import sys
import numpy as np

from mm_trajectories import timescaling, path, util


DT = 0.1
LX = 3.0
FREQUENCY = 2.0
R = 0.5


def main():
    rospy.init_node('trajectory_generator')

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30

    # wait until current pose is received
    p0, quat0 = util.wait_for_initial_pose(DT)

    scaling = timescaling.QuinticTimeScaling(duration / 2.0)
    # scaling = timescaling.QuinticTimeScaling(duration)
    # scaling = timescaling.CubicTimeScaling(duration)
    # scaling = timescaling.LinearTimeScaling(duration)
    # scaling = timescaling.TrapezoidalTimeScalingV(0.1, duration)
    traj1 = path.Rotational(p0, quat0, [0, 0, 1], 0.5*np.pi, scaling)
    quat1, _, _ = traj1.sample_rotation(np.array([duration / 2.0]))
    quat1 = np.squeeze(quat1)
    traj2 = path.Rotational(p0, quat1, [-1, 0, 0], 0.5*np.pi, scaling)

    waypoints = util.create_waypoints(traj1, duration / 2.0, DT)
    # waypoints = util.create_waypoints(traj1, duration, DT)
    waypoints += util.create_waypoints(traj2, duration / 2.0, DT)

    util.publish(waypoints, DT)

    print('Launched rotational trajectory with duration of {} seconds.'.format(duration))


if __name__ == '__main__':
    main()


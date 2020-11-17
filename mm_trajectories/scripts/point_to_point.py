#!/usr/bin/env python2

from __future__ import print_function

import rospy
import sys
import tf.transformations as tfs

from mm_msgs.msg import PoseTrajectory

import mm_kinematics.kinematics as kinematics

from mm_trajectories import trajectory
from mm_trajectories.util import JointInitializer


def main():
    rospy.init_node('trajectory_generator')
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30

    traj_pub = rospy.Publisher('/trajectory/poses', PoseTrajectory,
                               queue_size=10)

    # Need to wait a second between setting up the publisher and actually using
    # it to publish a message.
    rospy.sleep(1.0)

    dt = 0.1

    # wait until current joint state is received
    q0, dq0 = JointInitializer.wait_for_msg(dt)

    # calculate initial tool position and velocity
    T0 = kinematics.calc_w_T_tool(q0)
    p0 = tfs.translation_from_matrix(T0)
    quat0 = tfs.quaternion_from_matrix(T0)

    p1 = p0 + [3, 0, 0]

    timescaling = trajectory.QuinticTimeScaling(duration)
    traj = trajectory.PointToPoint(p0, p1, quat0, timescaling, duration)
    waypoints = trajectory.create_waypoints(traj, duration, dt)

    msg = PoseTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.points = waypoints
    msg.dt = rospy.Duration(dt)

    traj_pub.publish(msg)

    print('Launched point-to-point trajectory with duration of {} seconds.'.format(duration))


if __name__ == '__main__':
    main()


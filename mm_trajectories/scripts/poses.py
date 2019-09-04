#!/usr/bin/env python2

from __future__ import print_function

import rospy
import numpy as np
import tf.transformations as tfs

from mm_msgs.msg import PoseTrajectory

import mm_kinematics.kinematics as kinematics
import mm_msgs.conversions as conversions
from mm_trajectories.util import JointInitializer
from mm_trajectories.trajectory import StationaryTrajectory, LineTrajectory, SineTrajectory, RotationalTrajectory

import IPython


def launch_pose_traj():
    pose_traj_pub = rospy.Publisher('/trajectory/poses', PoseTrajectory,
                                    queue_size=10)

    # Need to wait a second between setting up the publisher and actually using
    # it to publish a message.
    rospy.sleep(1.0)

    dt = 0.1

    # wait until current joint state is received
    q0, dq0 = JointInitializer.wait_for_msg(dt)

    # calculate initial EE position and velocity
    T0 = kinematics.forward(q0)
    p0 = tfs.translation_from_matrix(T0)
    quat0 = tfs.quaternion_from_matrix(T0)

    print('Trajectory initialized with initial position\n= {}'.format(list(p0)))

    traj = LineTrajectory(p0, quat0)
    waypoints = []
    t = 0
    tf = 30

    N = int(tf / dt) + 1
    for _ in xrange(N):
        p, v = traj.sample_linear(t)
        q, w = traj.sample_rotation(t)

        waypoint = conversions.waypoint_msg(t, p, v, q, w)

        waypoints.append(waypoint)

        t += dt

    msg = PoseTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.points = waypoints
    msg.dt = rospy.Duration(dt)

    pose_traj_pub.publish(msg)


def main():
    rospy.init_node('trajectory_generator')
    launch_pose_traj()


if __name__ == '__main__':
    main()


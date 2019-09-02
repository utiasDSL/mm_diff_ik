#!/usr/bin/env python2

from __future__ import print_function

import rospy
import numpy as np
import tf.transformations as tfs

from mm_msgs.msg import PoseTrajectoryPoint, PoseTrajectory

import mm_kinematics.kinematics as kinematics
from mm_trajectories import JointInitializer, StationaryTrajectory, LineTrajectory, SineTrajectory, RotationalTrajectory

import IPython


def build_waypoint(t, p, v, q, w):
    waypoint = PoseTrajectoryPoint()
    waypoint.time_from_start = rospy.Time(t)

    waypoint.pose.position.x = p[0]
    waypoint.pose.position.y = p[1]
    waypoint.pose.position.z = p[2]

    waypoint.velocity.linear.x = v[0]
    waypoint.velocity.linear.y = v[1]
    waypoint.velocity.linear.z = v[2]

    waypoint.pose.orientation.x = q[0]
    waypoint.pose.orientation.y = q[1]
    waypoint.pose.orientation.z = q[2]
    waypoint.pose.orientation.w = q[3]

    waypoint.velocity.angular.x = w[0]
    waypoint.velocity.angular.y = w[1]
    waypoint.velocity.angular.z = w[2]

    return waypoint


def launch_pose_traj():
    pose_traj_pub = rospy.Publisher('/pose_traj', PoseTrajectory, queue_size=10)

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

    while t < tf:
        p, v = traj.sample_linear(t)
        q, w = traj.sample_rotation(t)

        waypoint = build_waypoint(t, p, v, q, w)

        waypoints.append(waypoint)

        t += dt

    msg = PoseTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.points = waypoints
    msg.dt = rospy.Duration(dt)

    pose_traj_pub.publish(msg)


def launch_point_traj():
    ''' Launch a trajectory to a single point in space. '''
    pose_traj_pub = rospy.Publisher('/pose_traj', PoseTrajectory, queue_size=10)
    rospy.sleep(1.0)

    tf = 5

    q0, _ = JointInitializer.wait_for_msg(0.1)
    qf = np.zeros(9)
    # TODO get this from real platform

    T0 = kinematics.forward(q0)
    p0 = tfs.translation_from_matrix(T0)
    quat0 = tfs.quaternion_from_matrix(T0)

    Tf = kinematics.forward(qf)
    pf = tfs.translation_from_matrix(Tf)
    quatf = tfs.quaternion_from_matrix(Tf)

    wp1 = build_waypoint(0,  p0, np.zeros(3), quat0, np.zeros(3))
    wp2 = build_waypoint(tf, pf, np.zeros(3), quatf, np.zeros(3))

    msg = PoseTrajectory()
    msg.points = [wp1, wp2]
    msg.dt = rospy.Duration(tf)
    pose_traj_pub.publish(msg)


def launch_pose_cmds():
    pose_cmd_pub = rospy.Publisher('/pose_cmd', PoseTrajectoryPoint, queue_size=10)

    dt = 0.1

    # wait until current joint state is received
    q0, dq0 = JointInitializer.wait_for_msg(dt)

    # calculate initial EE position and velocity
    T0 = kinematics.forward(q0)
    p0 = tfs.translation_from_matrix(T0)
    quat0 = tfs.quaternion_from_matrix(T0)

    print('Trajectory initialized with initial position\n= {}'.format(list(p0)))

    traj = LineTrajectory(p0, quat0)

    while not rospy.is_shutdown():
        now = rospy.get_time()
        p, v = traj.sample_linear(now)
        q, w = traj.sample_rotation(now)

        waypoint = PoseTrajectoryPoint()

        waypoint.pose.position.x = p[0]
        waypoint.pose.position.y = p[1]
        waypoint.pose.position.z = p[2]

        # Constant orientation for now.
        waypoint.pose.orientation.x = q[0]
        waypoint.pose.orientation.y = q[1]
        waypoint.pose.orientation.z = q[2]
        waypoint.pose.orientation.w = q[3]

        waypoint.velocity.linear.x = v[0]
        waypoint.velocity.linear.y = v[1]
        waypoint.velocity.linear.z = v[2]

        waypoint.velocity.angular.x = w[0]
        waypoint.velocity.angular.y = w[1]
        waypoint.velocity.angular.z = w[2]

        waypoint.time_from_start = rospy.Time(dt)

        pose_cmd_pub.publish(waypoint)

        print(waypoint)

        rospy.sleep(dt)


def main():
    rospy.init_node('trajectory_generator')
    launch_pose_traj()
    # launch_point_traj()


if __name__ == '__main__':
    main()


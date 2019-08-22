#!/usr/bin/env python2

from __future__ import print_function

import rospy
import numpy as np
import tf.transformations as tfs

from mm_msgs.msg import PoseTrajectoryPoint

from mm_trajectories import JointInitializer, StationaryTrajectory, LineTrajectory, SineTrajectory
from mm_kinematics import ThingKinematics


def main():
    rospy.init_node('trajectory_generator')
    pose_cmd_pub = rospy.Publisher('/pose_cmd', PoseTrajectoryPoint, queue_size=10)

    dt = 0.1

    # wait until current joint state is received
    q0, dq0 = JointInitializer.wait_for_msg(dt)

    # calculate initial EE position and velocity
    kin = ThingKinematics()
    T0 = kin.calc_fk(q0)
    p0 = tfs.translation_from_matrix(T0)
    quat0 = tfs.quaternion_from_matrix(T0)

    print('Trajectory initialized with initial position\n= {}'.format(list(p0)))

    traj = StationaryTrajectory(p0, quat0)

    while not rospy.is_shutdown():
        now = rospy.get_time()
        p, v = traj.sample(now)

        waypoint = PoseTrajectoryPoint()

        waypoint.pose.position.x = p[0]
        waypoint.pose.position.y = p[1]
        waypoint.pose.position.z = p[2]

        # Constant orientation for now.
        waypoint.pose.orientation.x = traj.quat0[0]
        waypoint.pose.orientation.y = traj.quat0[1]
        waypoint.pose.orientation.z = traj.quat0[2]
        waypoint.pose.orientation.w = traj.quat0[3]

        waypoint.velocity.linear.x = v[0]
        waypoint.velocity.linear.y = v[1]
        waypoint.velocity.linear.z = v[2]

        waypoint.velocity.angular.x = 0
        waypoint.velocity.angular.y = 0
        waypoint.velocity.angular.z = 0

        waypoint.time_from_start = rospy.Time(dt)

        pose_cmd_pub.publish(waypoint)

        print(waypoint)

        rospy.sleep(dt)


if __name__ == '__main__':
    main()


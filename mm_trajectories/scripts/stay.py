#!/usr/bin/env python2

from __future__ import print_function

import rospy
import numpy as np
import tf.transformations as tfs

from geometry_msgs.msg import PoseStamped

import mm_kinematics.kinematics as kinematics
import mm_msgs.conversions as conversions
from mm_trajectories.util import JointInitializer

import IPython


def launch_stationary_traj():
    ''' Launch a trajectory to a single point in space. '''
    pub = rospy.Publisher('/trajectory/point', PoseStamped, queue_size=10)
    rospy.sleep(1.0)

    q0, _ = JointInitializer.wait_for_msg(0.1)

    T0 = kinematics.calc_w_T_tool(q0)
    p0 = tfs.translation_from_matrix(T0)
    quat0 = tfs.quaternion_from_matrix(T0)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose = conversions.pose_msg(p0, quat0)

    pub.publish(msg)


def main():
    rospy.init_node('trajectory_generator')
    launch_stationary_traj()


if __name__ == '__main__':
    main()


#!/usr/bin/env python2
"""Launch a trajectory to a single point in space."""

from __future__ import print_function

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped

import mm_msgs.conversions as conversions
from mm_trajectories import util


DT = 0.1
OFFSET = np.array([4, 0, 0])  # offset from current position


def main():
    rospy.init_node('trajectory_generator')
    pub = rospy.Publisher('/trajectory/point', PoseStamped, queue_size=10)
    rospy.sleep(1.0)

    p0, quat0 = util.wait_for_initial_pose(DT)

    p = p0 + OFFSET

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose = conversions.pose_msg(p, quat0)

    pub.publish(msg)

    print('Launched trajectory to push point.')


if __name__ == '__main__':
    main()


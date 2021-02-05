#!/usr/bin/env python2
from __future__ import print_function

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from mm_msgs.msg import Obstacle, Obstacles

import IPython


R_bl = np.eye(2)
p_lb_b = np.array([0.393, 0])


def rotation_matrix(angle):
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, -s], [s, c]])


def extract_positions(scan):
    """Extract positions relative to the LIDAR from the LaserScan message
       scan.
    """
    n = len(scan.ranges)
    ranges = np.array(scan.ranges)
    angles = np.array([scan.angle_min + i*scan.angle_increment for i in range(n)])

    valid_idx, = np.nonzero(np.logical_and(ranges >= scan.range_min, ranges <= scan.range_max))

    positions = np.vstack((np.cos(angles), np.sin(angles))) * ranges

    return positions[:, valid_idx]


class ObstacleDetector(object):
    def __init__(self):
        self.q = np.zeros(9)
        self.scan = None

        self.state_sub = rospy.Subscriber('/mm_joint_states', JointState, self.joint_state_cb)
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_cb)
        self.obstacle_pub = rospy.Publisher('/obstacles', Obstacles, queue_size=10)

    def publish(self):
        if not self.scan:
            return

        p_ol_ls = extract_positions(self.scan)

        # get everything in the world frame
        p_bw_w = self.q[:2]
        R_wb = rotation_matrix(self.q[2])
        p_lw_w = p_bw_w + R_wb.dot(p_lb_b)
        p_ow_ws = p_lw_w[:, None] + R_wb.dot(R_bl).dot(p_ol_ls)

        # TODO: need to cluster into objects
        # naive: cluster all nearby points into circles---how to find these efficiently

    def joint_state_cb(self, msg):
        self.q = msg.position

    def scan_cb(self, msg):
        self.scan = msg


def main():
    rospy.init_node('mm_lidar_viewer')

    rate = rospy.Rate(10)
    detector = ObstacleDetector()

    while not rospy.is_shutdown():
        detector.publish()
        rate.sleep()


if __name__ == '__main__':
    main()

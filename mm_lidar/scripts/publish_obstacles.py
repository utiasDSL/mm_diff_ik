#!/usr/bin/env python2
from __future__ import print_function

import rospy
import numpy as np
from scipy.spatial import distance, KDTree

from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from mm_msgs.msg import Obstacle, Obstacles

import mm_lidar
from mm_lidar import util

import IPython


R_bl = np.eye(2)
p_lb_b = np.array([0.393, 0])

HZ = 100
RADIUS = 0.1


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

        p_ol_ls, valid_mask = mm_lidar.extract_positions(self.scan)

        # ignore last one
        p_ol_ls = p_ol_ls[:, :-1]
        valid_mask = valid_mask[:-1]

        # convert frames
        p_bw_w = self.q[:2]
        R_wb = util.rotation_matrix(self.q[2])
        p_ob_bs = p_lb_b[:, None] + R_bl.dot(p_ol_ls)
        p_ow_ws = p_bw_w[:, None] + R_wb.dot(p_ob_bs)

        p_ow_ws_closest = mm_lidar.filter_points_by_distance(p_ob_bs, p_ow_ws, valid_mask)

        # we can try additional filtering if desired
        # p_ow_ws_valid = p_ow_ws[:, valid_mask]
        # closest_filtered = []
        # tree = KDTree(p_ow_ws_valid.T)
        # clusters = tree.query_ball_point(p_ow_ws_closest.T, 0.1)
        # for cluster in clusters:
        #     if len(cluster) < 4:
        #         continue
        #     p = np.mean(p_ow_ws_valid[:, cluster], axis=1)
        #     closest_filtered.append(p)
        # closest_filtered = np.array(closest_filtered).T

        msg = Obstacles()
        msg.header.stamp = rospy.Time.now()
        for i in xrange(p_ow_ws_closest.shape[1]):
            obs = Obstacle()
            obs.centre.x = p_ow_ws_closest[0, i]
            obs.centre.y = p_ow_ws_closest[1, i]
            obs.centre.z = 0
            obs.radius = RADIUS
            msg.obstacles.append(obs)

        self.obstacle_pub.publish(msg)

    def joint_state_cb(self, msg):
        self.q = np.array(msg.position)

    def scan_cb(self, msg):
        self.scan = msg


def main():
    rospy.init_node('mm_lidar_publisher')

    rate = rospy.Rate(HZ)
    detector = ObstacleDetector()

    while not rospy.is_shutdown():
        detector.publish()
        rate.sleep()


if __name__ == '__main__':
    main()

#!/usr/bin/env python2
from __future__ import print_function

import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance, KDTree

from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan

import mm_lidar
from mm_lidar import util

import IPython


R_bl = np.eye(2)
p_lb_b = np.array([0.393, 0])

# don't detect anything beyond this distance from base
INFLUENCE_DIST = 2.0


class LidarDetector(object):
    def __init__(self):
        self.q = np.zeros(9)
        self.scan = None

        self._init_plot()

        self.state_sub = rospy.Subscriber(
            "/mm/joint_states", JointState, self.joint_state_cb
        )
        self.scan_sub = rospy.Subscriber("/front/scan", LaserScan, self.scan_cb)

    def _init_plot(self):
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlim(-2, 3)
        self.ax.set_ylim(-3, 1)
        self.ax.grid()

        (self.detection_plot,) = self.ax.plot([], [], "o", color="k")
        (self.base_origin_plot,) = self.ax.plot([], [], "o", color="r")
        (self.lidar_plot,) = self.ax.plot([], [], "o", color="r")
        (self.filtered_plot,) = self.ax.plot([], [], "o", color="g")

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def update(self):
        if not self.scan:
            return

        # p_ol_ls = mm_lidar.extract_positions(self.scan)
        p_ol_ls, valid_mask = mm_lidar.extract_positions(self.scan)

        # ignore last one
        # p_ol_ls = p_ol_ls[:, :-1]
        # valid_mask = valid_mask[:-1]

        # convert frames
        p_bw_w = self.q[:2]
        R_wb = util.rotation_matrix(self.q[2])
        p_lw_w = p_bw_w + R_wb.dot(p_lb_b)
        p_ob_bs = p_lb_b[:, None] + R_bl.dot(p_ol_ls)
        p_ow_ws = p_bw_w[:, None] + R_wb.dot(p_ob_bs)
        p_ow_ws_valid = p_ow_ws[:, valid_mask]

        p_ow_ws_closest = mm_lidar.filter_points_by_distance(
            p_ob_bs, p_ow_ws, valid_mask
        )

        # we can try additional filtering if desired
        # closest_filtered = []
        # tree = KDTree(p_ow_ws_valid.T)
        # clusters = tree.query_ball_point(p_ow_ws_closest.T, 0.1)
        # for cluster in clusters:
        #     if len(cluster) < 4:
        #         continue
        #     p = np.mean(p_ow_ws_valid[:, cluster], axis=1)
        #     closest_filtered.append(p)
        # closest_filtered = np.array(closest_filtered).T

        self.base_origin_plot.set_xdata([p_bw_w[0]])
        self.base_origin_plot.set_ydata([p_bw_w[1]])

        self.lidar_plot.set_xdata([p_lw_w[0]])
        self.lidar_plot.set_ydata([p_lw_w[1]])

        self.detection_plot.set_xdata(p_ow_ws[0, :])
        self.detection_plot.set_ydata(p_ow_ws[1, :])

        self.filtered_plot.set_xdata(p_ow_ws_closest[0, :])
        self.filtered_plot.set_ydata(p_ow_ws_closest[1, :])
        # self.filtered_plot.set_xdata(closest_filtered[0, :])
        # self.filtered_plot.set_ydata(closest_filtered[1, :])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def joint_state_cb(self, msg):
        self.q = np.array(msg.position)

    def scan_cb(self, msg):
        self.scan = msg


def main():
    rospy.init_node("mm_lidar_viewer")

    rate = rospy.Rate(10)
    detector = LidarDetector()

    while not rospy.is_shutdown():
        detector.update()
        rate.sleep()


if __name__ == "__main__":
    main()

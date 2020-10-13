#!/usr/bin/env python2
# Align the desired Vicon marker constellation origin with the Vicon room
# origin. To be done before calibrating the zero pose using the Vicon bridge
# package.
import rospy
import numpy as np

from vicon_bridge.msg import Markers
from geometry_msgs.msg import Twist


HZ = 10
K = 0.5 * np.eye(3)  # proportional gain
MAX_V = 0.2
EPS = 0.001  # break when error norm smaller than this

ORIGIN_MARKER = 'ThingBase21'
ORIENTATION_MARKER = 'ThingBase22'


def find_marker(name, markers):
    for marker in markers:
        if marker.marker_name == name:
            return marker
    return None


class BaseViconCalibrationNode(object):
    ''' Detect non-constellation vicon markers and republish as obstacles. '''
    def __init__(self):
        self.obstacles = []
        self.marker_sub = rospy.Subscriber('/vicon/markers', Markers,
                                           self.marker_cb)
        self.base_vel_pub = rospy.Publisher('/ridgeback_velocity_controller/cmd_vel',
                                            Twist, queue_size=10)
        self.initialized = False
        self.markers = []

    def marker_cb(self, msg):
        self.markers = msg.markers
        self.initialized = True

    def calc_cmd(self):
        marker1 = find_marker(ORIGIN_MARKER, self.markers)
        marker2 = find_marker(ORIENTATION_MARKER, self.markers)

        # first marker should be aligned with the origin
        xe = marker1.translation.x * 0.001
        ye = marker1.translation.y * 0.001

        # vector between the two markers should be aligned with the x-axis
        dx = marker2.translation.x - marker1.translation.x
        dy = marker2.translation.y - marker1.translation.y
        te = np.arctan2(dy, dx)

        err = np.array([xe, ye, te])  # error vector

        # calculate velocity command, bounding the result
        cmd = np.minimum(np.maximum(K * err, -MAX_V), MAX_V)

        return cmd, err

    def loop(self, hz):
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
            if self.initialized:
                cmd, err = self.calc_cmd()
                if np.linalg.norm(err) < EPS:
                    rospy.loginfo('Set point reached.')
                    break
                msg = Twist()
                msg.linear.x = self.cmd[0]
                msg.linear.y = self.cmd[1]
                msg.angular.z = self.cmd[2]
                self.base_vel_pub.publish(msg)

            rate.sleep()


def main():
    rospy.init_node('base_vicon_calibration_node')
    node = BaseViconCalibrationNode()
    node.loop(HZ)


if __name__ == '__main__':
    main()

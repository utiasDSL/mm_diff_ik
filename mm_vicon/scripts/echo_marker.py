#!/usr/bin/env python2
import sys
import rospy
from vicon_bridge.msg import Markers


HZ = 10

ORIGIN_MARKER = 'ThingEE1'
# ORIENTATION_MARKER = 'ThingBase22'


def find_marker(name, markers):
    for marker in markers:
        if marker.marker_name == name:
            return marker
    return None


class MarkerEchoNode(object):
    ''' Detect non-constellation vicon markers and republish as obstacles. '''
    def __init__(self, marker_name):
        self.obstacles = []
        self.marker_sub = rospy.Subscriber('/vicon/markers', Markers,
                                           self.marker_cb)
        self.marker_name = marker_name
        self.initialized = False
        self.markers = []

    def marker_cb(self, msg):
        self.markers = msg.markers
        self.initialized = True

    def loop(self, hz):
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
            if self.initialized:
                marker1 = find_marker(self.marker_name, self.markers)
                print(marker1)
            rate.sleep()


def main():
    marker_name = sys.argv[1]
    rospy.init_node('base_vicon_calibration_node')
    node = MarkerEchoNode(marker_name)
    node.loop(HZ)


if __name__ == '__main__':
    main()

#!/usr/bin/env python2
import rospy
import numpy as np
from vicon_bridge.msg import Markers

np.set_printoptions(precision=3, suppress=True)


HZ = 10

MARKER_NAMES = ['TrayTest1', 'TrayTest2', 'TrayTest3']


def find_markers(names, markers):
    ''' Extract markers with given names from array of markers. '''
    desired_markers = []
    for marker in markers:
        if marker.marker_name in names:
            desired_markers.append(marker)
    return desired_markers


def calc_normal(markers):
    ''' Calculate the normal to a plane containing three markers. '''
    p1 = np.array([markers[0].translation.x, markers[0].translation.y, markers[0].translation.z])
    p2 = np.array([markers[1].translation.x, markers[1].translation.y, markers[1].translation.z])
    p3 = np.array([markers[2].translation.x, markers[2].translation.y, markers[2].translation.z])

    normal = np.cross(p2 - p1, p3 - p1)
    normal = normal / np.linalg.norm(normal)

    # always take the normal to be positive in z-direction
    # NOTE fine for current experiments, but does not handle flipping the tray
    if normal[2] < 0:
        normal = -1 * normal
    return normal


def angle_between(a, b):
    ''' Calculate the angle between two vectors a and b. '''
    alen = np.linalg.norm(a)
    blen = np.linalg.norm(b)
    angle = np.arccos(a.dot(b) / (alen * blen))
    return angle


class TrayPlaneNode(object):
    def __init__(self):
        self.obstacles = []
        self.marker_sub = rospy.Subscriber('/vicon/markers', Markers,
                                           self.marker_cb)
        self.markers = []

    def marker_cb(self, msg):
        self.markers = msg.markers

    def loop(self, hz):
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
            markers = find_markers(MARKER_NAMES, self.markers)
            if len(markers) >= 3:
                normal = calc_normal(markers)
                floor_normal = np.array([0, 0, 1])
                angle = angle_between(normal, floor_normal)
                print('rad = {:.3f} deg = {:.3f}'.format(angle, np.rad2deg(angle)))
            rate.sleep()


def main():
    rospy.init_node('tray_node')
    node = TrayPlaneNode()
    node.loop(HZ)


if __name__ == '__main__':
    main()

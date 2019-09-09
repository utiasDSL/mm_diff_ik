#!/usr/bin/env python2
import rospy

from vicon_bridge.msg import Markers
from mm_msgs.msg import Obstacle, Obstacles


# Radius around obstacle centres
R = 0.1
HZ = 10


class Square(object):
    def __init__(self, c, r):
        ''' Initialize with centre c and radius (half side length) r. '''
        self.c = c
        self.r = r

    def contains(self, p):
        ''' Returns True if point p is contained within the square, False
            otherwise. '''
        if p[0] > self.c[0] + self.r:
            return False
        elif p[0] < self.c[0] - self.r:
            return False
        elif p[1] > self.c[1] + self.r:
            return False
        elif p[1] < self.c[1] - self.r:
            return False
        return True


# For obstacles to register, they must be within all INCLUDE squares but not in
# any EXCLUDE squares.
INCLUDES = [Square([0, 0], 4)]
EXCLUDES = []


class ObstacleDetectionNode(object):
    ''' Detect non-constellation vicon markers and republish as obstacles. '''
    def __init__(self):
        self.obstacles = []
        self.marker_sub = rospy.Subscriber('/vicon/markers', Markers,
                                           self.marker_cb)
        self.obstacle_pub = rospy.Publisher('/obstacles', Obstacles,
                                            queue_size=10)

    def marker_cb(self, msg):
        # Only take unnamed markers (i.e. not part of the Thing constellation)
        markers = filter(lambda marker: marker.marker_name == '', msg.markers)

        new_obstacles = []

        for marker in markers:
            obstacle = Obstacle()
            obstacle.radius = R

            # Markers have units in millimetres: convert to metres.
            x = 0.001 * marker.translation.x
            y = 0.001 * marker.translation.y
            z = 0.001 * marker.translation.z

            # Obstacle must be within all include squares but not in any
            # exclude squares.
            for square in INCLUDES:
                if not square.contains([x, y]):
                    continue
            for square in EXCLUDES:
                if square.contains([x, y]):
                    continue

            obstacle.centre.x = x
            obstacle.centre.y = y
            obstacle.centre.z = z

            new_obstacles.append(obstacle)

        # Sort obstacles so that order remains stable between Vicon frames (not
        # required for the controller but nice for viewing). Sort is ascending
        # in x coordinate.
        self.obstacles = sorted(new_obstacles, key=lambda o: o.centre.x)

    def loop(self, hz):
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
            msg = Obstacles()
            msg.obstacles = self.obstacles
            msg.header.stamp = rospy.Time.now()
            self.obstacle_pub.publish(msg)

            rate.sleep()


def main():
    rospy.init_node('obstacle_detection')
    rospy.loginfo('Obstacle detection started.')
    node = ObstacleDetectionNode()
    node.loop(HZ)


if __name__ == '__main__':
    main()

#!/usr/bin/env python2
import rospy

from vicon_bridge.msg import Markers
from mm_msgs.msg import Obstacle, Obstacles


# Radius around obstacle centres
R = 0.3
HZ = 10


class ObstacleDetectionNode(object):
    ''' Detect non-constellation vicon markers and republish as obstacles. '''
    def __init__(self):
        self.marker_sub = rospy.Subscriber('/vicon/markers', Markers,
                                           self.marker_cb)
        self.obstacle_pub = rospy.Publisher('/obstacles', Obstacles,
                                            queue_size=10)

    def marker_cb(self, msg):
        # Only take unnamed markers (i.e. not part of the Thing constellation)
        markers = filter(lambda marker: marker.marker_name == '', msg.markers)

        msg = Obstacles()

        for marker in markers:
            obstacle = Obstacle()
            obstacle.radius = R

            # Markers have units in millimetres: convert to metres.
            obstacle.centre.x = 0.001 * marker.translation.x
            obstacle.centre.y = 0.001 * marker.translation.y
            obstacle.centre.z = 0.001 * marker.translation.z

            msg.obstacles.append(obstacle)

        msg.header.stamp = rospy.Time.now()
        self.obstacle_pub.publish(msg)

    def loop(self, hz):
        # Just spin since we do everything in the callback
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            rate.sleep()


def main():
    rospy.init_node('obstacle_detection')
    rospy.loginfo('Obstacle detection started.')
    node = ObstacleDetectionNode()
    node.loop(HZ)


if __name__ == '__main__':
    main()

#!/usr/bin/env python2
import rospy

from mm_msgs.msg import Obstacle, Obstacles


# Radius around obstacle centres
R = 0.3
HZ = 100


class ObstacleDetectionNodeSim(object):
    ''' Detect non-constellation vicon markers and republish as obstacles. '''
    def __init__(self, obstacles):
        self.obstacle_pub = rospy.Publisher('/obstacles', Obstacles,
                                            queue_size=10)
        self.obstacles = obstacles

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

    o1 = Obstacle()
    o1.centre.x = 1.0
    o1.centre.y = 0.0
    o1.radius   = R

    obstacles = [o1]

    rospy.loginfo('Simulated obstacle detection started.')
    node = ObstacleDetectionNodeSim(obstacles)
    node.loop(HZ)


if __name__ == '__main__':
    main()

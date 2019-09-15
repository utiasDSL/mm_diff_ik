#!/usr/bin/env python2
import rospy

from std_msgs.msg import Empty
from mm_msgs.msg import Obstacle, Obstacles


# Radius around obstacle centres
R = 0.1
HZ = 100

# Original obstacles: (0, -0.5), (1, 0.5)


# TODO these should really just be a single class
class StaticObstacle(object):
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def msg(self, t):
        o = Obstacle()
        o.centre.x = self.x
        o.centre.y = self.y
        o.radius   = self.r
        return o


class DynamicObstacle(object):
    def __init__(self, t0, x0, y0, vx, vy, r):
        self.t0 = t0
        self.x0 = x0
        self.y0 = y0
        self.vx = vx
        self.vy = vy
        self.r = r

    def msg(self, t):
        t = t - self.t0
        o = Obstacle()
        o.centre.x = self.x0 + self.vx * t
        o.centre.y = self.y0 + self.vy * t
        o.radius   = self.r
        return o


class ObstacleDetectionNodeSim(object):
    ''' Detect non-constellation vicon markers and republish as obstacles. '''
    def __init__(self, obstacles):
        self.launch_sub = rospy.Subscriber('/launch_obs', Empty, self.launch_cb)
        self.obstacle_pub = rospy.Publisher('/obstacles', Obstacles,
                                            queue_size=10)
        self.obstacles = obstacles

    def loop(self, hz):
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
            msg = Obstacles()
            msg.obstacles = [obs.msg(rospy.get_time()) for obs in self.obstacles]
            msg.header.stamp = rospy.Time.now()
            self.obstacle_pub.publish(msg)

            rate.sleep()

    def launch_cb(self, msg):
        # o = DynamicObstacle(2.0, -0.5, 0.0, 0.1, R)
        o1 = DynamicObstacle(rospy.get_time(), 0.5, -0.5, -0.1, 0, R)
        o2 = DynamicObstacle(rospy.get_time(), 1.5, 0.5, -0.1, 0, R)
        self.obstacles = [o1, o2]
        print('Launched dynamic obstacles')


def main():
    rospy.init_node('obstacle_detection')

    o1 = StaticObstacle(-1.0, -0.5, R)
    o2 = StaticObstacle( 0.0,  0.5, R)
    o3 = StaticObstacle( 1.0, -0.5, R)
    o4 = StaticObstacle( 2.0,  0.5, R)

    obstacles = [o1, o2, o3, o4]
    # obstacles = []

    rospy.loginfo('Simulated obstacle detection started.')
    node = ObstacleDetectionNodeSim(obstacles)
    node.loop(HZ)


if __name__ == '__main__':
    main()

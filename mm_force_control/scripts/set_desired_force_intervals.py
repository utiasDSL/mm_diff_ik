#!/usr/bin/env python2
import rospy

from std_msgs.msg import Float64


START = -5
INCREMENT = -5


def wait_and_count(n):
    rate = rospy.Rate(1.0)
    seconds = 0
    while not rospy.is_shutdown() and seconds < n:
        seconds += 1
        print(n - seconds)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('mm_force_des_node')

    force_des_pub = rospy.Publisher('/force/desired', Float64, queue_size=10)

    rospy.sleep(1.0)
    force = START

    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = force
        force_des_pub.publish(msg)
        print('Published desired force = {}'.format(force))

        wait_and_count(20)
        force += INCREMENT

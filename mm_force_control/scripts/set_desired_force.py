#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float64


DESIRED_FORCE = 10


if __name__ == '__main__':
    rospy.init_node('mm_force_des_node')
    pub = rospy.Publisher('/force/desired', Float64, queue_size=1)

    rospy.sleep(1.0)

    msg = Float64()
    msg.data = DESIRED_FORCE
    pub.publish(msg)

    rospy.sleep(0.1)

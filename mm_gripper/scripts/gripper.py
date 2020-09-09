#!/usr/bin/env python2
import sys

import rospy
from robotiq_s_model_control.msg import SModel_robot_output


if __name__ == '__main__':
    open_ = len(sys.argv) > 1 and sys.argv[1][0] == 'o'

    rospy.init_node('mm_gripper_node')
    pub = rospy.Publisher('SModelRobotOutput', SModel_robot_output, queue_size=10)

    rospy.sleep(1.0)

    msg = SModel_robot_output()
    msg.rACT = 1
    msg.rMOD = 1  # 1 for pinched; 0 for unpinched
    msg.rGTO = 1
    msg.rATR = 0
    msg.rICF = 0
    if open_:
        msg.rPRA = 0
    else:
        msg.rPRA = 255
    msg.rSPA = 255
    msg.rFRA = 100

    pub.publish(msg)

    rospy.sleep(1.0)
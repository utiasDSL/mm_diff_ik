#!/usr/bin/env python2
# usage:
#  rosrun mm_gripper gripper.py o|c [delay]
import sys

import rospy
from robotiq_s_model_control.msg import SModel_robot_output


if __name__ == '__main__':
    open_ = len(sys.argv) > 1 and sys.argv[1][0] == 'o'

    if len(sys.argv) > 2:
        delay = float(sys.argv[2])
    else:
        delay = 0

    rospy.init_node('mm_gripper_node')
    pub = rospy.Publisher('SModelRobotOutput', SModel_robot_output, queue_size=10)

    rospy.sleep(1.0 + delay)

    msg = SModel_robot_output()
    msg.rACT = 1

    # 0 for normal, 1 for pinched, 2 for wide mode, 3 for scissor mode
    # NOTE scissor mode may not be working
    msg.rMOD = 2

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

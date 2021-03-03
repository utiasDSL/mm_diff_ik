#!/usr/bin/env python2
"""Send the robot to a home configuration.

The joint control node must be running.

Configuation is specified by name from the list in config/home.yaml
"""
import os
import sys
import rospy
import rospkg
import numpy as np
import yaml

from mm_msgs.msg import JointControllerInfo
from trajectory_msgs.msg import JointTrajectoryPoint

HZ = 125

CONFIG_FILE = os.path.join("config", "home.yaml")


class HomeMonitor(object):
    def __init__(self, qd):
        self.error = np.ones(9)
        self.info_sub = rospy.Subscriber(
            "/mm/control/joint/info", JointControllerInfo, self.info_cb
        )

    def info_cb(self, msg):
        self.error = np.array(msg.error.positions)

    def loop(self, hz):
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            if (np.abs(self.error) < 1e-3).all():
                break
            rate.sleep()


def main():
    rospy.init_node("home_node")

    rospack = rospkg.RosPack()
    config_path = os.path.join(rospack.get_path("mm_control"), CONFIG_FILE)

    try:
        config_name = sys.argv[1]
    except IndexError:
        print(
            "Name of desired home configuration is required. Options can be found in {}".format(
                config_path
            )
        )
        return

    with open(config_path) as f:
        config = yaml.load(f)

    qdb = np.array(config["base"])
    qda = np.array(config["arm"][config_name])

    # make sure configs are the right shape
    assert qdb.shape == (3,)
    assert qda.shape == (6,)

    qd = np.concatenate((qdb, qda))

    rospy.loginfo("Going home...")

    point_pub = rospy.Publisher(
        "/mm/control/joint/point", JointTrajectoryPoint, queue_size=1
    )
    rospy.sleep(1.0)
    msg = JointTrajectoryPoint()
    msg.positions = list(qd)
    point_pub.publish(msg)

    monitor = HomeMonitor(qd)
    monitor.loop(HZ)

    rospy.loginfo("Done")


if __name__ == "__main__":
    main()

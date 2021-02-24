#!/usr/bin/env python2
"""Send the robot to a home configuration.

Configuation is specified by name from the list in config/home.yaml
"""
import os
import sys
import rospy
import rospkg
import numpy as np
import yaml

from mm_kinematics import KinematicModel
from mm_math_util import bound_array
from mm_control.control import MMController

HZ = 125

# Proportional gain matrix.
K = 0.5 * np.eye(9)

# Maximum joint speed.
MAX_U = 0.2

CONFIG_FILE = os.path.join("config", "home.yaml")


class JointController(MMController):
    """Simple joint-space controller.

    Takes a desired joint configuration and moves toward it using proportional
    control.
    """
    def __init__(self, qd):
        self.qd = qd
        self.model = KinematicModel()
        super(JointController, self).__init__()

    def loop(self, hz):
        """Control loop."""
        rate = rospy.Rate(hz)

        # wait until a joint message is received
        while not rospy.is_shutdown() and not self.joint_state_rec:
            rate.sleep()

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            e = self.qd - self.q
            if (np.abs(e) < 1e-3).all():
                break

            Binv = self.model.calc_joint_input_map_inv(self.q)
            u = Binv.dot(K).dot(e)
            self.u = bound_array(u, -MAX_U, MAX_U)

            self.publish_joint_speeds(now)

            rate.sleep()


def main():
    rospy.init_node("home_node")

    rospack = rospkg.RosPack()
    config_path = os.path.join(rospack.get_path("mm_control"), CONFIG_FILE)

    try:
        config_name = sys.argv[1]
    except IndexError:
        print("Name of desired home configuration is required. Options can be found in {}".format(config_path))
        return

    with open(config_path) as f:
        config = yaml.load(f)

    qdb = np.array(config["base"])
    qda = np.array(config["arm"][config_name])

    assert(qdb.shape == (3,))
    assert(qda.shape == (6,))

    qd = np.concatenate((qdb, qda))

    rospy.loginfo("Going home...")

    controller = JointController(qd)
    controller.loop(HZ)

    rospy.loginfo("Done")


if __name__ == "__main__":
    main()

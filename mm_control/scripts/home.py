#!/usr/bin/env python2
import rospy
import numpy as np

from mm_kinematics import KinematicModel
from mm_math_util import bound_array
from mm_control.control import MMController

HZ = 125

# Proportional gain matrix.
K = 0.5 * np.eye(9)

# Maximum joint speed.
MAX_U = 0.2

# Default home position
DEFAULT_HOME = np.array([
    -1.0,
    0.0,
    0.5*np.pi,
    0.0,
    -0.75 * np.pi,
    -0.5 * np.pi,
    -0.75 * np.pi,
    -0.5 * np.pi,
    0.5 * np.pi,
])


class JointController(MMController):
    def __init__(self, qd):
        self.qd = qd
        self.model = KinematicModel()
        super(JointController, self).__init__()

    def loop(self, hz):
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

    rospy.loginfo("Going home...")

    # TODO load as map from yaml file... or even just keep it here
    controller = JointController(DEFAULT_HOME)
    controller.loop(HZ)

    rospy.loginfo("Done")


if __name__ == "__main__":
    main()

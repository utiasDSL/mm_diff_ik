#!/usr/bin/env python2
"""Send the robot to a home configuration.

The joint control node must be running.

Configuation is specified by name from the list in config/home.yaml
"""
from datetime import datetime
import json
import os

import rospy
import rospkg
import numpy as np
import yaml

from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState


HZ = 10

CONFIG_FILE = os.path.join("config", "calibration_configs.yaml")

JOINT_CONTROL_INFO_TOPIC = "/mm/control/joint/info"
JOINT_CONTROL_SETPOINT_TOPIC = "/mm/control/joint/point"
VICON_EE_TOPIC = "/vicon/ThingEE/ThingEE"
JOINT_STATE_TOPIC = "/mm/joint_states"


def quaternion_average(Q):
    """Compute average of n quaternions stored in n*4 matrix Q."""
    e, v = np.linalg.eig(Q.T.dot(Q))
    max_idx = np.argmax(e)
    if (e >= e[max_idx]).any():
        rospy.logwarn("quaternion average: non-unique max eigenvalue!")
    return v[:, max_idx]


def approx_quaternion_average(Q):
    """Compute approximate average of n quaterions.

    This average is a good approximation when the quaternions are similar.
    """
    q = np.mean(Q, axis=0)
    return q / np.linalg.norm(q)


class CalibrationDataListener(object):
    """Loop until error is small."""

    def __init__(self):
        self.actual = np.zeros(9)
        self.received_joint_state = False
        self.received_vicon = False

        self.vicon_sub = rospy.Subscriber(
            VICON_EE_TOPIC, TransformStamped, self.vicon_cb
        )
        self.joint_sub = rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self.joint_cb)

    def vicon_cb(self, msg):
        r = msg.transform.translation
        Q = msg.transform.rotation

        self.ee_position = np.array([r.x, r.y, r.z])
        self.ee_quaternion = np.array([Q.x, Q.y, Q.z, Q.w])

        self.received_vicon = True

    def joint_cb(self, msg):
        self.actual = np.array(msg.position)
        self.received_joint_state = True

    def has_large_error(self, qd, threshold=1e-3):
        """Returns True if measured joints are all within threshold of desired."""
        return (np.abs(qd - self.actual) > threshold).any()

    def ready(self):
        """Returns True when listener is ready to be used."""
        return self.received_joint_state and self.received_vicon

    def average_sample(self, n_sample=10, dt=0.5):
        """Average n_sample pairs of (q, T), measured every dt seconds."""
        qs = np.zeros((n_sample, 9))
        rs = np.zeros((n_sample, 3))
        quats = np.zeros((n_sample, 4))

        for i in xrange(n_sample):
            qs[i, :] = self.actual
            rs[i, :] = self.ee_position
            quats[i, :] = self.ee_quaternion
            rospy.sleep(dt)

        q_avg = np.mean(qs, axis=0)
        r_avg = np.mean(rs, axis=0)
        quat_avg = approx_quaternion_average(quats)

        return q_avg, r_avg, quat_avg


def main():
    rospy.init_node("calibration_data_collection_node")

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("mm_control")
    config_path = os.path.join(pkg_path, CONFIG_FILE)

    with open(config_path) as f:
        config = yaml.load(f)

    arm_configs = np.array(config["arm"])
    assert arm_configs.shape[1] == 6  # six joints
    n_config = arm_configs.shape[0]  # number of configurations

    point_pub = rospy.Publisher(
        JOINT_CONTROL_SETPOINT_TOPIC, JointTrajectoryPoint, queue_size=1
    )
    listener = CalibrationDataListener()
    rospy.sleep(1.0)

    rate = rospy.Rate(HZ)

    # nominal base configuration is where the robot is at the time (so we can
    # perform calibration starting anywhere)
    rospy.loginfo("Waiting to receive joint state and Vicon measurement.")
    while not listener.ready():
        rate.sleep()
    base_config = listener.actual[:3]

    data = []

    for i in range(n_config):
        rospy.loginfo("Going to point {} of {}...".format(i + 1, n_config))

        qd = np.concatenate((base_config, arm_configs[i, :]))

        msg = JointTrajectoryPoint()
        msg.positions = list(qd)
        point_pub.publish(msg)

        # wait until controller has converged to the desired location
        while listener.has_large_error(qd):
            if rospy.is_shutdown():
                return
            rate.sleep()

        rospy.loginfo(
            "Arrived at point {} of {}. Collecting data...".format(i + 1, n_config)
        )

        q_avg, r_avg, quat_avg = listener.average_sample()

        data.append(
            {
                "joint_angles": list(q_avg),
                "ee_position": list(r_avg),
                "ee_quaternion": list(quat_avg),
            }
        )

        rospy.loginfo("Done")

    # save the recorded data
    data_file_name = (
        "calibration_data_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".json"
    )
    data_path = os.path.join(pkg_path, "data", data_file_name)
    with open(data_path, "w") as f:
        json.dump(data, f, indent=2)

    rospy.loginfo("Wrote data to {}.".format(data_path))


if __name__ == "__main__":
    main()

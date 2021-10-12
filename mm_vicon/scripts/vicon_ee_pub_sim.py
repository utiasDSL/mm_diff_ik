#!/usr/bin/env python2
"""Publish fake transform of EE for simulation, based on kinematic model."""
import rospy
import numpy as np
import tf.transformations as tfs

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

from mm_kinematics import KinematicModel


HZ = 100

JOINT_STATE_TOPIC = "/mm/joint_states"
VICON_EE_TOPIC = "/vicon/ThingEE/ThingEE"


class ViconEESimulator(object):
    def __init__(self, model):
        self.model = model
        self.received_joint_state = False

        self.joint_sub = rospy.Subscriber(
            JOINT_STATE_TOPIC, JointState, self.joint_cb
        )
        self.vicon_pub = rospy.Publisher(
            VICON_EE_TOPIC, TransformStamped, queue_size=1
        )

    def joint_cb(self, msg):
        self.q = np.array(msg.position)
        self.received_joint_state = True

    def publish(self):
        if self.received_joint_state:
            T_we = self.model.calc_T_w_ee(self.q)
            r_ew_w = tfs.translation_from_matrix(T_we)
            Q_we = tfs.quaternion_from_matrix(T_we)

            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()

            msg.transform.translation.x = r_ew_w[0]
            msg.transform.translation.y = r_ew_w[1]
            msg.transform.translation.z = r_ew_w[2]

            msg.transform.rotation.x = Q_we[0]
            msg.transform.rotation.y = Q_we[1]
            msg.transform.rotation.z = Q_we[2]
            msg.transform.rotation.w = Q_we[3]

            self.vicon_pub.publish(msg)


def main():
    rospy.init_node("vicon_ee_pub_sim_node")

    model = KinematicModel()
    simulator = ViconEESimulator(model)
    rate = rospy.Rate(HZ)

    while not rospy.is_shutdown():
        simulator.publish()
        rate.sleep()
    pass


if __name__ == "__main__":
    main()

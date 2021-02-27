#!/usr/bin/env python2

from __future__ import print_function

import rospy
import numpy as np
import tf.transformations as tfs

from sensor_msgs.msg import JointState

import mm_msgs.conversions as conversions
from mm_kinematics import KinematicModel
from mm_msgs.msg import CartesianTrajectory


def create_waypoints(traj, duration, dt):
    """Create waypoints for the trajectory."""
    N = int(duration / dt) + 1
    ts = np.linspace(0, duration, N)

    # sample the trajectory
    ps, vs, as_ = traj.sample_linear(ts)
    qs, ws, alphas = traj.sample_rotation(ts)

    # convert to waypoint messages
    waypoints = []
    for i in xrange(N):
        waypoint = conversions.waypoint_msg(
            ts[i], ps[i, :], vs[i, :], as_[i, :], qs[i, :], ws[i, :], alphas[i, :]
        )
        waypoints.append(waypoint)

    return waypoints


def publish(waypoints, dt):
    """Publish the waypoints."""
    traj_pub = rospy.Publisher("/trajectory/poses", CartesianTrajectory, queue_size=10)

    # Need to wait a second between setting up the publisher and actually using
    # it to publish a message.
    rospy.sleep(1.0)

    msg = CartesianTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.points = waypoints

    traj_pub.publish(msg)


def wait_for_initial_pose(dt):
    """Wait until initial pose is received."""
    model = KinematicModel()

    # wait until current joint state is received
    q0, dq0 = JointInitializer.wait_for_msg(dt)

    # calculate initial tool position and velocity
    T0 = model.calc_T_w_tool(q0)
    p0 = tfs.translation_from_matrix(T0)
    quat0 = tfs.quaternion_from_matrix(T0)

    return p0, quat0


class JointInitializer(object):
    """Initialize joints by waiting for a joint state message."""

    def __init__(self):
        self.joint_state_sub = rospy.Subscriber(
            "/mm_joint_states", JointState, self.joint_state_cb
        )
        self.initialized = False

    def joint_state_cb(self, msg):
        self.q0 = np.array(msg.position)
        self.dq0 = np.array(msg.velocity)

        # Once we have a message, we don't need to listen any more
        self.joint_state_sub.unregister()
        self.initialized = True

    def state(self):
        return self.q0, self.dq0

    @staticmethod
    def wait_for_msg(dt):
        j = JointInitializer()
        while not rospy.is_shutdown() and not j.initialized:
            rospy.sleep(dt)
        return j.state()

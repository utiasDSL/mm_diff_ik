#!/usr/bin/env python2
import rospy
import numpy as np

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist

from mm_math_util import bound_array, rotation2d


SIM_RATE = 250.0  # Hz
SIM_DT = 1.0 / SIM_RATE

PUB_RATE = 125.0
PUB_DT = 1.0 / PUB_RATE

HOME_CONFIG = np.array([
    0,
    0,
    0,
    0,
    -0.75 * np.pi,
    -0.5 * np.pi,
    -0.75 * np.pi,
    -0.5 * np.pi,
    0.5 * np.pi,
])

BASE_VEL_LIM = np.array([1.0, 1.0, 2.0])
ARM_VEL_LIM = np.array([2.16, 2.16, 3.15, 3.2, 3.2, 3.2])

BASE_ACC_LIM = np.ones(3)
ARM_ACC_LIM = 8.0 * np.ones(6)


class RobotSim(object):
    """Simulation of Thing platform: Ridgeback omnidirectional base + UR10
    6-DOF arm."""

    def __init__(self, q):
        # base
        self.qb = q[:3]
        self.dqb = np.zeros(3)
        self.ddqb = np.zeros(3)
        self.ub = np.zeros(3)
        self.ub_prev = np.zeros(3)

        # arm
        self.qa = q[3:]
        self.dqa = np.zeros(6)
        self.ddqa = np.zeros(6)
        self.ua = np.zeros(6)
        self.ua_prev = np.zeros(6)

        # publish joint states
        self.rb_state_pub = rospy.Publisher(
            "/rb_joint_states", JointState, queue_size=10
        )
        self.ur10_state_pub = rospy.Publisher(
            "/ur10_joint_states", JointState, queue_size=10
        )
        self.state_pub = rospy.Publisher("/mm_joint_states", JointState, queue_size=10)

        # subscribe to joint speed commands
        self.rb_joint_speed_sub = rospy.Subscriber(
            "/ridgeback_velocity_controller/cmd_vel", Twist, self.rb_joint_speed_cb
        )
        self.ur10_joint_speed_sub = rospy.Subscriber(
            "/ur_driver/joint_speed", JointTrajectory, self.ur10_joint_speed_cb
        )

    def _calc_dqb(self, qb, ub):
        R_wb = rotation2d(qb[2])
        skew = np.array([[0, -ub[2]], [ub[2], 0]])
        dqb = np.zeros(3)
        dqb[:2] = R_wb.dot(self.ub[:2])  # + skew.dot(qb[:2])
        dqb[2] = ub[2]
        return dqb

    def rb_joint_speed_cb(self, msg):
        """Callback for velocity commands for the base."""
        # msg is of type geometry_msgs/Twist
        self.ub_prev = self.ub
        self.ub = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def ur10_joint_speed_cb(self, msg):
        """Callback for velocity commands to the arm joints."""
        # msg is of type trajectory_msgs/JointTrajectory
        self.ua_prev = self.ua
        self.ua = np.array(msg.points[0].velocities)

    def _limit_velocity(self, v, limit, quiet=False):
        v_bounded = bound_array(v, -limit, limit)
        if not np.allclose(v, v_bounded) and not quiet:
            rospy.logwarn("Velocity limits violated.")
        return v_bounded

    def _calc_acceleration(self, v, v_prev, dt, limit=None, quiet=False):
        a = (v - v_prev) / dt

        if limit:
            a_bounded = bound_array(a, -limit, limit)
            if not np.allclose(a, a_bounded) and not quiet:
                rospy.logwarn("Acceleration limits violated.")
            return a_bounded
        else:
            return a

    def step(self, dt):
        """Step the sim forward in time."""
        # apply velocity limits
        self.ua = self._limit_velocity(self.ua, ARM_VEL_LIM)
        self.ub = self._limit_velocity(self.ub, BASE_VEL_LIM)

        # TODO acceleration limits

        self.dqb = self._calc_dqb(self.qb, self.ub)
        self.dqa = self.ua

        self.qb = self.qb + dt * self.dqb
        self.qa = self.qa + dt * self.dqa

    def publish_joint_states(self):
        """Publish current joint states (position and velocity)."""
        now = rospy.Time.now()

        q = np.concatenate((self.qb, self.qa))
        dq = np.concatenate((self.dqb, self.dqa))
        ddq = np.concatenate((self.ddqb, self.ddqa))

        # Base
        rb_joint_state = JointState()
        rb_joint_state.header.stamp = now
        rb_joint_state.position = list(self.qb)
        rb_joint_state.velocity = list(self.dqb)
        rb_joint_state.effort = list(self.ddqb)
        self.rb_state_pub.publish(rb_joint_state)

        # Arm
        ur10_joint_state = JointState()
        ur10_joint_state.header.stamp = now
        ur10_joint_state.position = list(self.qa)
        ur10_joint_state.velocity = list(self.dqa)
        ur10_joint_state.effort = list(self.ddqa)
        self.ur10_state_pub.publish(ur10_joint_state)

        # All joints together, for convenience
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.position = list(q)
        joint_state.velocity = list(dq)
        joint_state.effort = list(ddq)  # abuse of notation
        self.state_pub.publish(joint_state)


def main():
    rospy.init_node("mm_robot_sim")
    rate = rospy.Rate(SIM_RATE)

    sim = RobotSim(HOME_CONFIG)

    # publishing is on a separate timer than the sim loop
    def pub_cb(event):
        sim.publish_joint_states()

    rospy.Timer(rospy.Duration(PUB_DT), pub_cb)

    # simulation loop
    while not rospy.is_shutdown():
        sim.step(SIM_DT)
        rate.sleep()


if __name__ == "__main__":
    main()

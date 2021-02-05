#!/usr/bin/env python2
import rospy
import numpy as np

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist


SIM_RATE = 125.0  # Hz
DT = 1. / SIM_RATE

BASE_VEL_LIM = np.array([1.0, 1.0, 2.0])
ARM_VEL_LIM = np.array([2.16, 2.16, 3.15, 3.2, 3.2, 3.2])

BASE_ACC_LIM = np.ones(3)
ARM_ACC_LIM = 8. * np.ones(6)


def rotation2d(angle):
    """Principal rotation matrix about the z axis."""
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, -s],
                     [s,  c]])


def bound_array(a, lb, ub):
    return np.minimum(np.maximum(a, lb), ub)


class RobotSim(object):
    """Simulation of Thing platform: Ridgeback omnidirectional base + UR10
       6-DOF arm."""
    def __init__(self, q):
        self.last_time = rospy.get_time()
        self.ta = self.last_time
        self.tb = self.last_time

        self.qa = q[3:]
        self.qb = q[:3]
        self.dqa = np.zeros(6)
        self.ub = np.zeros(3)
        self.ddqa = np.zeros(6)
        self.ddqb = np.zeros(3)

        # publish joint states
        self.rb_state_pub = rospy.Publisher('/rb_joint_states', JointState, queue_size=10)
        self.ur10_state_pub = rospy.Publisher('/ur10_joint_states', JointState, queue_size=10)
        self.state_pub = rospy.Publisher('/mm_joint_states', JointState, queue_size=10)

        # subscribe to joint speed commands
        self.rb_joint_speed_sub = rospy.Subscriber(
                '/ridgeback_velocity_controller/cmd_vel', Twist,
                self.rb_joint_speed_cb)
        self.ur10_joint_speed_sub = rospy.Subscriber('/ur_driver/joint_speed',
                JointTrajectory, self.ur10_joint_speed_cb)

    def _calc_dqb(self, qb, ub):
        R_wb = rotation2d(qb[2])
        skew = np.array([[0, -ub[2]], [ub[2], 0]])
        dqb = np.zeros(3)
        dqb[:2] = R_wb.dot(self.ub[:2]) + skew.dot(qb[:2])
        dqb[2] = ub[2]
        return dqb

    def _integrate_qb(self, qb, ub, dt):
        dqb = self._calc_dqb(qb, ub)
        return qb + dt * dqb

    def rb_joint_speed_cb(self, msg):
        ''' Callback for velocity commands for the base. '''
        # msg is of type geometry_msgs/Twist

        # update time
        # RB controller doesn't give a timestamp with the message, so we
        # need to just use the current time
        tb = rospy.get_time()
        dt = tb - self.tb
        self.tb = tb

        # apply velocity limits
        ub = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        ub_bounded = bound_array(ub, -BASE_VEL_LIM, BASE_VEL_LIM)
        if not np.allclose(ub, ub_bounded):
            rospy.logwarn('Base velocity constraints violated.')

        # calculate acceleration
        ddqb = (ub_bounded - self.ub) / dt
        ddqb_bounded = bound_array(ddqb, -BASE_ACC_LIM, BASE_ACC_LIM)
        if not np.allclose(ddqb, ddqb_bounded):
            rospy.logwarn('Base acceleration constraints violated.')

        self.ub = ub_bounded
        self.ddqb = ddqb_bounded

        # integrate from last time
        self.qb = self._integrate_qb(self.qb, self.ub, dt)

    def ur10_joint_speed_cb(self, msg):
        ''' Callback for velocity commands to the arm joints. '''
        # msg is of type trajectory_msgs/JointTrajectory

        # update time
        ta = rospy.get_time()  #msg.header.stamp.to_sec()
        dt = ta - self.ta
        self.ta = ta

        # apply velocity limits
        dqa = np.array(msg.points[0].velocities)
        dqa_bounded = bound_array(dqa, -ARM_VEL_LIM, ARM_VEL_LIM)
        if not np.allclose(dqa, dqa_bounded):
            rospy.logwarn('Arm velocity constraints violated.')

        # calculate acceleration
        ddqa = (dqa_bounded - self.dqa) / dt
        ddqa_bounded = bound_array(ddqa, -ARM_ACC_LIM, ARM_ACC_LIM)
        if not np.allclose(ddqa, ddqa_bounded):
            rospy.logwarn('Arm acceleration constraints violated.')

        self.dqa = dqa_bounded
        self.ddqa = ddqa_bounded

        # integrate from last time message received to when this message
        # was
        self.qa += dt * self.dqa

    def publish_joint_states(self):
        ''' Publish current joint states (position and velocity). '''
        now = rospy.Time.now()
        t = now.to_sec()

        # integrate to get best estimate at the current time
        qa = self.qa + (t - self.ta) * self.dqa
        qb = self._integrate_qb(self.qb, self.ub, t - self.tb)

        # use integrated joint angle to calculate current base velocity
        dqb = self._calc_dqb(qb, self.ub)

        q = np.concatenate((qb, qa))
        dq = np.concatenate((dqb, self.dqa))
        ddq = np.concatenate((self.ddqb, self.ddqa))

        # Base
        rb_joint_state = JointState()
        rb_joint_state.header.stamp = now
        rb_joint_state.position = list(qb)
        rb_joint_state.velocity = list(dqb)
        rb_joint_state.effort = list(self.ddqb)
        self.rb_state_pub.publish(rb_joint_state)

        # Arm
        ur10_joint_state = JointState()
        ur10_joint_state.header.stamp = now
        ur10_joint_state.position = list(qa)
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
    rospy.init_node('mm_robot_sim')
    rate = rospy.Rate(SIM_RATE)

    q = np.zeros(9)
    q = np.array([0, 0, 0, 0, -0.75*np.pi, -0.5*np.pi, -0.75*np.pi, -0.5*np.pi, 0.5*np.pi])
    # q[4] = -np.pi*0.75
    # q[5] = -np.pi/2

    sim = RobotSim(q)

    while not rospy.is_shutdown():
        sim.publish_joint_states()
        rate.sleep()


if __name__ == '__main__':
    main()

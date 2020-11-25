#!/usr/bin/env python2
import rospy
import numpy as np
from threading import Lock

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist


PUB_RATE = 125.0
SIM_RATE = 125.0  # Hz
DT = 1. / SIM_RATE

VEL_LIM = np.array([1.0, 1.0, 2.0, 2.16, 2.16, 3.15, 3.2, 3.2, 3.2])
ACC_LIM = np.array([1, 1, 1, 8, 8, 8, 8, 8, 8])


def bound_array(a, lb, ub):
    return np.minimum(np.maximum(a, lb), ub)


class RobotSim(object):
    ''' Simulation of Thing platform: Ridgeback omnidirectional base + UR10
        6-DOF arm. '''
    def __init__(self, q):
        # maintain an internal robot state
        self.q = q
        self.dq = np.zeros(q.shape)
        self.dq_last = np.zeros_like(q)
        self.ddq = np.zeros(q.shape)

        self.last_time = rospy.get_time()
        self.ta = self.last_time
        self.tb = self.last_time

        self.qa = q[3:]
        self.qb = q[:3]
        self.dqa = np.zeros(6)
        self.dqb = np.zeros(3)

        self.lock = Lock()

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

        # self.pub_timer = rospy.Timer(rospy.Duration(1. / PUB_RATE), self.publish_joint_states)

    def rb_joint_speed_cb(self, msg):
        ''' Callback for velocity commands for the base. '''
        # print('Received RB speeds = {}'.format(msg.linear))
        # dq_base = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        # with self.lock:
        #     self.dq_last[:3] = self.dq[:3]
        #     self.dq[:3] = dq_base

        with self.lock:
            # update time
            # RB controller doesn't give a timestamp with the message, so we
            # need to just use the current time
            tb = rospy.get_time()
            dt = tb - self.tb
            self.tb = tb

            # integrate from last time
            self.qb += dt * self.dqb

            self.dqb = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

        # self.ddq[:3] = (dq_base - self.dq[:3]) * 125.0
        # self.q[:3] += 0.008 * dq_base

    def ur10_joint_speed_cb(self, msg):
        ''' Callback for velocity commands to the arm joints. '''
        # msg is of type trajectory_msgs/JointTrajectory
        # take the velocities from the first point

        with self.lock:
            # update time
            ta = rospy.get_time()  #msg.header.stamp.to_sec()
            dt = ta - self.ta
            self.ta = ta

            # integrate from last time message received to when this message
            # was
            self.qa += dt * self.dqa

            self.dqa = np.array(msg.points[0].velocities)

        # dq_arm = np.array(msg.points[0].velocities)
        # with self.lock:
        #     self.dq_last[3:] = self.dq[3:]
        #     self.dq[3:] = dq_arm
        # self.ddq[3:] = (dq_arm - self.dq[3:]) * 125.0
        # self.q[3:] += 0.008 * dq_arm

    def publish_joint_states(self):
        ''' Publish current joint states (position and velocity). '''

        # locking not needed here since we're only reading values

        with self.lock:
            now = rospy.Time.now()
            t = now.to_sec()

            # integrate to get best estimate at the current time
            qa = self.qa + (t - self.ta) * self.dqa
            qb = self.qb + (t - self.tb) * self.dqb

            print(t - self.ta)
            print(t - self.tb)

            q = np.concatenate((qb, qa))
            dq = np.concatenate((self.dqb, self.dqa))

            # Base
            rb_joint_state = JointState()
            rb_joint_state.header.stamp = now
            rb_joint_state.position = list(qb)
            rb_joint_state.velocity = list(self.dqb)
            self.rb_state_pub.publish(rb_joint_state)

            # Arm
            ur10_joint_state = JointState()
            ur10_joint_state.header.stamp = now
            ur10_joint_state.position = list(qa)
            ur10_joint_state.velocity = list(self.dqa)
            self.ur10_state_pub.publish(ur10_joint_state)

            # All joints together, for convenience (e.g. for simulation)
            joint_state = JointState()
            joint_state.header.stamp = now
            joint_state.position = list(q)
            joint_state.velocity = list(dq)
            # joint_state.effort = list(self.ddq)  # abuse of notation
            self.state_pub.publish(joint_state)

    def step(self):
        # now = rospy.get_time()
        # dt = now - self.last_time
        # self.last_time = now

        # # velocity limits
        # dq1 = bound_array(self.dq, -VEL_LIM, VEL_LIM)
        #
        # # TODO it would be better to enforce these when messages are received
        # if (dq1 - self.dq).dot(dq1 - self.dq) > 1e-8:
        #     rospy.loginfo('Joint velocity limits hit.')
        #
        # # acceleration limits
        # dq2 = bound_array(dq1, -ACC_LIM * DT + self.dq_last, ACC_LIM * DT + self.dq_last)
        #
        # if not (dq2 == dq1).all():
        #     rospy.loginfo('Joint acceleration limits hit.')

        # Integrate to get current joint angles.
        # with self.lock:
        #     self.q = self.q + DT * self.dq

        self.publish_joint_states()


def main():
    rospy.init_node('mm_robot_sim')
    rate = rospy.Rate(SIM_RATE)

    q = np.zeros(9)
    q = np.array([0, 0, 0, 0, -0.75*np.pi, -0.5*np.pi, -0.75*np.pi, -0.5*np.pi, 0.5*np.pi])
    # q[4] = -np.pi*0.75
    # q[5] = -np.pi/2

    sim = RobotSim(q)

    while not rospy.is_shutdown():
        sim.step()
        rate.sleep()


if __name__ == '__main__':
    main()

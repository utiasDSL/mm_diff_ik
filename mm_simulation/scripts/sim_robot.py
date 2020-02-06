#!/usr/bin/env python2
import rospy
import numpy as np
import time

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist


class RobotSim(object):
    ''' Simulation of Thing platform: Ridgeback omnidirectional base + UR10
        6-DOF arm. '''
    def __init__(self, q, dq):
        # maintain an internal robot state
        self.q = q
        self.dq = dq
        self.last_time = time.time()

        self.fh = open('/home/adam/dl/log.txt', 'w')

        # publish joint states
        self.rb_state_pub = rospy.Publisher('/rb_joint_states', JointState, queue_size=10)
        self.ur10_state_pub = rospy.Publisher('/ur10_joint_states', JointState, queue_size=10)
        self.state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # subscribe to joint speed commands
        self.rb_joint_speed_sub = rospy.Subscriber(
                '/ridgeback_velocity_controller/cmd_vel', Twist,
                self.rb_joint_speed_cb)
        self.ur10_joint_speed_sub = rospy.Subscriber('/ur_driver/joint_speed',
                JointTrajectory, self.ur10_joint_speed_cb)

    def rb_joint_speed_cb(self, msg):
        ''' Callback for velocity commands for the base. '''
        # print('Received RB speeds = {}'.format(msg.linear))
        self.dq[:3] = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def ur10_joint_speed_cb(self, msg):
        ''' Callback for velocity commands to the arm joints. '''
        # msg is of type trajectory_msgs/JointTrajectory
        # take the velocities from the first point
        # print('Received UR10 speeds = {}'.format(msg.points[0].velocities))
        self.dq[3:] = np.array(msg.points[0].velocities)

    def publish_joint_states(self):
        ''' Publish current joint states (position and velocity). '''
        # Base
        rb_joint_state = JointState()
        rb_joint_state.position = list(self.q[:3])
        rb_joint_state.velocity = list(self.dq[:3])
        self.rb_state_pub.publish(rb_joint_state)

        # Arm
        ur10_joint_state = JointState()
        ur10_joint_state.position = list(self.q[3:])
        ur10_joint_state.velocity = list(self.dq[3:])
        self.ur10_state_pub.publish(ur10_joint_state)

        # All joints together, for convenience (e.g. for simulation)
        joint_state = JointState()
        joint_state.position = list(self.q)
        joint_state.velocity = list(self.dq)
        self.state_pub.publish(joint_state)

    def step(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Integrate to get current joint angles.
        self.q += dt * self.dq

        max_dq = np.max(self.dq)
        min_dq = np.min(self.dq)
        if max_dq > min_dq:
            self.fh.write('{}, {}\n'.format(max_dq, min_dq))

        self.publish_joint_states()


def main():
    rospy.init_node('mm_robot_sim')
    dt = 0.01

    q = np.zeros(9)
    dq = np.zeros(9)

    q[4] = -np.pi*0.75
    q[5] = -np.pi/2

    sim = RobotSim(q, dq)

    while not rospy.is_shutdown():
        sim.step()
        rospy.sleep(dt)
    sim.fh.close()


if __name__ == '__main__':
    main()

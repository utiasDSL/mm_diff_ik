#!/usr/bin/env python2

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from mm_msgs.msg import PoseTrajectoryPoint

from kinematics import ThingKinematics


class JointInitializer(object):
    def __init__(self):
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState,
                                                self.joint_state_cb)
        self.initialized = False

    def joint_state_cb(self, msg):
        self.q0 = np.array(msg.position)
        self.dq0 = np.array(msg.velocity)
        self.joint_state_sub.unregister()
        self.initialized = True

    def waiting(self):
        return not self.initialized

    def state(self):
        return self.q0, self.dq0


class LineTrajectory(object):
    ''' Straight line in x-direction. '''
    def __init__(self, p0):
        self.p0 = p0
        self.t0 = rospy.get_time()

    def sample(self, t):
        # 1cm/second
        x = self.p0[0] + 0.01 * (t - self.t0)
        y = self.p0[1]
        z = self.p0[2]

        dx = 0.01
        dy = dz = 0

        return np.array([x, y, z]), np.array([dx, dy, dz])


def main():
    # TODO need to get initial position
    rospy.init_node('traj_generator')
    pose_cmd_pub = rospy.Publisher('/pose_cmd', PoseTrajectoryPoint, queue_size=10)
    joint_init = JointInitializer()

    dt = 0.1

    # wait until current joint state is received
    while joint_init.waiting():
        rospy.sleep(dt)
    q0, dq0 = joint_init.state()


    kin = ThingKinematics()

    # calculate initial EE position and velocity
    T0 = kin.calc_fk(q0[:3], q0[3:])
    p0 = T0[:3,3]

    print('Trajectory initialized with initial position\n= {}'.format(list(p0)))

    traj = LineTrajectory(p0)

    while not rospy.is_shutdown():
        now = rospy.get_time()
        p, v = traj.sample(now)

        waypoint = PoseTrajectoryPoint()

        waypoint.pose.position.x = p[0]
        waypoint.pose.position.y = p[1]
        waypoint.pose.position.z = p[2]

        waypoint.velocity.linear.x = v[0]
        waypoint.velocity.linear.y = v[1]
        waypoint.velocity.linear.z = v[2]

        waypoint.time_from_start = rospy.Time(dt)

        pose_cmd_pub.publish(waypoint)

        rospy.sleep(dt)


if __name__ == '__main__':
    main()


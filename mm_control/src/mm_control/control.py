"""Python base controller."""
import numpy as np
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MMController(object):
    """Base controller class."""
    def __init__(self):
        self.q = np.zeros(9)
        self.dq = np.zeros(9)
        self.u = np.zeros(9)

        self.joint_state_rec = False

        self.joint_states_sub = rospy.Subscriber(
            "/mm_joint_states", JointState, self.joint_states_cb
        )

        self.ur10_joint_vel_pub = rospy.Publisher(
            "/ur_driver/joint_speed", JointTrajectory, queue_size=1
        )

        self.rb_joint_vel_pub = rospy.Publisher(
            "/ridgeback_velocity_controller/cmd_vel", Twist, queue_size=1
        )

    def publish_joint_speeds(self, now):
        # TODO need to use the base input mapping in the controller
        # c = np.cos(self.q[2])
        # s = np.sin(self.q[2])
        # R_wb = np.array([[c, -s], [s, c]])

        base_twist = Twist()
        base_twist.linear.x = self.u[0]
        base_twist.linear.y = self.u[1]
        base_twist.angular.z = self.u[2]

        arm_traj_point = JointTrajectoryPoint()
        arm_traj_point.velocities = list(self.u[3:])

        arm_traj = JointTrajectory()
        arm_traj.header.stamp = now
        arm_traj.points.append(arm_traj_point)

        self.ur10_joint_vel_pub.publish(arm_traj)
        self.rb_joint_vel_pub.publish(base_twist)

    def joint_states_cb(self, msg):
        self.q = np.array(msg.position)
        self.dq = np.array(msg.velocity)
        self.joint_state_rec = True

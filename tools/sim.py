import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import quadprog
import time

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from rr_msgs.msg import PoseTrajectoryPoint

from kinematics import ThingKinematics, R_t_from_T

import IPython


class RobotPlotter(object):
    def __init__(self):
        pass

    def start(self, Ts):
        ''' Launch the plot. '''
        plt.ion()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([0, 2])

        w_p_b, w_p_e, xs, ys, zs = self._calc_data(Ts)

        self.line, = self.ax.plot(xs, ys, zs=zs, marker='o')
        self.ax.plot([0], [0], zs=[0], c='b', marker='o', label='Origin')
        self.base_pt, = self.ax.plot([w_p_b[0]], [w_p_b[1]], zs=[w_p_b[2]],
                                     c='g', marker='o', label='Base')
        self.ee_pt, = self.ax.plot([w_p_e[0]], [w_p_e[1]], zs=[w_p_e[2]],
                                   c='r', marker='o', label='EE')

        self.ax.legend()

    def _calc_data(self, Ts):
        w_T_b = Ts[3]
        w_T_e = Ts[-1]

        _, w_p_b = R_t_from_T(w_T_b)
        _, w_p_e = R_t_from_T(w_T_e)

        xs = [T[0,3] for T in Ts[4:]]
        ys = [T[1,3] for T in Ts[4:]]
        zs = [T[2,3] for T in Ts[4:]]

        return w_p_b, w_p_e, xs, ys, zs

    def update(self, Ts):
        ''' Update plot based on new transforms Ts. '''
        w_p_b, w_p_e, xs, ys, zs = self._calc_data(Ts)

        self.line.set_xdata(xs)
        self.line.set_ydata(ys)
        self.line.set_3d_properties(zs)

        self.base_pt.set_xdata([w_p_b[0]])
        self.base_pt.set_ydata([w_p_b[1]])
        self.base_pt.set_3d_properties([w_p_b[2]])

        self.ee_pt.set_xdata([w_p_e[0]])
        self.ee_pt.set_ydata([w_p_e[1]])
        self.ee_pt.set_3d_properties([w_p_e[2]])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


class RobotSim(object):
    def __init__(self, q, dq):
        # maintain an internal robot state
        self.q = q
        self.dq = dq
        self.kin = ThingKinematics()
        self.Ts = self.kin.calc_fk_chain(q[:3], q[3:])
        self.last_time = time.time()

        self.base_state_pub = rospy.publisher('/rb_joint_states', JointState, queue_size=10)
        self.arm_state_pub = rospy.Publisher('/ur10_joint_states', JointState, queue_size=10)
        self.pose_cmd_pub = rospy.Publisher('/pose_cmd', PoseTrajectoryPoint, queue_size=10)
        self.joint_speed_sub = rospy.Subscriber('/ur_driver/joint_speed', JointTrajectory, self.joint_speed_cb)

    def joint_speed_cb(self, msg):
        # msg is of type trajectory_msgs/JointTrajectory
        # take the velocities from the first point
        self.dq = np.array(msg.points[0].velocities)

    def step(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Integrate to get current joint angles.
        self.q += dt * self.dq

        # Forward kinematics along the whole chain.
        self.Ts = self.kin.calc_fk_chain(self.q[:3], self.q[3:])


def main():
    rospy.init_node('robot_sim')

    sim = RobotSim(q=np.zeros(9), dq=np.zeros(9))

    plot = RobotPlotter()
    plot.start(sim.Ts)

    while not rospy.is_shutdown():
        sim.step()
        plot.update(sim.Ts)
        rospy.sleep(0.05)


if __name__ == '__main__':
    main()

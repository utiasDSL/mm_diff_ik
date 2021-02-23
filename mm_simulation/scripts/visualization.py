#!/usr/bin/env python2
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations as tfs
from scipy.linalg import null_space

from sensor_msgs.msg import JointState

from mm_msgs.msg import Obstacles, PoseTrajectory
from mm_kinematics import KinematicModel


class RobotPlotter(object):
    def __init__(self):
        self.joint_state_sub = rospy.Subscriber('/mm_joint_states', JointState,
                                                self._joint_state_cb)

        self.obstacle_sub = rospy.Subscriber('/obstacles', Obstacles,
                                             self._obstacle_cb)

        self.traj_sub = rospy.Subscriber('/trajectory/poses', PoseTrajectory,
                                         self._traj_cb)

        self.model = KinematicModel()

    def start(self, q0):
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

        self.q = q0
        self.obstacles = []
        self.x_traj = []
        self.y_traj = []
        self.z_traj = []
        self.traj_changed = False

        w_p_b, w_p_e, w_p_t, xs, ys, zs = self._calc_arm_positions()
        x_obs, y_obs, z_obs = self._calc_obs_positions()

        self.line, = self.ax.plot(xs, ys, zs=zs, marker='o')
        self.ax.plot([0], [0], zs=[0], c='k', marker='o', label='Origin')
        self.base_pt, = self.ax.plot([w_p_b[0]], [w_p_b[1]], zs=[w_p_b[2]],
                                     c='g', marker='o', label='Base')
        self.ee_pt, = self.ax.plot([w_p_e[0], w_p_t[0]], [w_p_e[1], w_p_t[1]],
                                   zs=[w_p_e[2], w_p_t[2]], c='r', marker='o',
                                   label='EE/Tool')

        # draws a plane that was used as an obstacle for force control
        # xp = 1.5
        # p0 = [xp, 0, 0]
        # n0 = [-1, -1, 0]
        # n0 = n0 / np.linalg.norm(n0)
        # V = null_space([n0])
        # v0 = V[:, 0]
        # v1 = V[:, 1]
        #
        # p1 = p0 + v0
        # p2 = p0 + v0 + v1
        # p3 = p0 - v0 + v1
        # p4 = p0 - v0
        #
        # xs = [p1[0], p2[0], p3[0], p4[0], p1[0]]
        # ys = [p1[1], p2[1], p3[1], p4[1], p1[1]]
        # zs = [p1[2], p2[2], p3[2], p4[2], p1[2]]
        #
        # self.plane, = self.ax.plot(xs, ys, zs=zs, color='k')

        self.obs_plot, = self.ax.plot(x_obs, y_obs, zs=z_obs, marker='o', c='k')

        self.traj_plot, = self.ax.plot(self.x_traj, self.y_traj, zs=self.z_traj, c='k')

        self.ax.legend()

    def _joint_state_cb(self, msg):
        # TODO may need a lock here
        self.q = np.array(msg.position)

    def _obstacle_cb(self, msg):
        self.obstacles = msg.obstacles

    def _traj_cb(self, msg):
        self.x_traj = [point.pose.position.x for point in msg.points]
        self.y_traj = [point.pose.position.y for point in msg.points]
        self.z_traj = [point.pose.position.z for point in msg.points]
        self.traj_changed = True

    def _calc_arm_positions(self):
        Ts = self.model.calc_chain(self.q)

        w_T_b = self.model.calc_T_w_base(self.q)
        w_T_e = self.model.calc_T_w_ee(self.q)
        w_T_t = self.model.calc_T_w_tool(self.q)

        w_p_b = tfs.translation_from_matrix(w_T_b)
        w_p_e = tfs.translation_from_matrix(w_T_e)
        w_p_t = tfs.translation_from_matrix(w_T_t)

        xs = [T[0, 3] for T in Ts[3:]]
        ys = [T[1, 3] for T in Ts[3:]]
        zs = [T[2, 3] for T in Ts[3:]]

        return w_p_b, w_p_e, w_p_t, xs, ys, zs

    def _calc_obs_positions(self):
        x_obs = [obs.centre.x for obs in self.obstacles]
        y_obs = [obs.centre.y for obs in self.obstacles]
        z_obs = [obs.centre.z for obs in self.obstacles]

        return x_obs, y_obs, z_obs

    def refresh(self):
        ''' Update plot based on current transforms. '''
        w_p_b, w_p_e, w_p_t, xs, ys, zs = self._calc_arm_positions()
        x_obs, y_obs, z_obs = self._calc_obs_positions()

        self.line.set_xdata(xs)
        self.line.set_ydata(ys)
        self.line.set_3d_properties(zs)

        self.base_pt.set_xdata([w_p_b[0]])
        self.base_pt.set_ydata([w_p_b[1]])
        self.base_pt.set_3d_properties([w_p_b[2]])

        self.ee_pt.set_xdata([w_p_e[0], w_p_t[0]])
        self.ee_pt.set_ydata([w_p_e[1], w_p_t[1]])
        self.ee_pt.set_3d_properties([w_p_e[2], w_p_t[2]])

        self.obs_plot.set_xdata(x_obs)
        self.obs_plot.set_ydata(y_obs)
        self.obs_plot.set_3d_properties(z_obs)

        if self.traj_changed:
            self.traj_plot.set_xdata(self.x_traj)
            self.traj_plot.set_ydata(self.y_traj)
            self.traj_plot.set_3d_properties(self.z_traj)
            self.traj_changed = False

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main():
    rospy.init_node('mm_visualization')

    plot = RobotPlotter()
    plot.start(q0=np.zeros(9))

    while not rospy.is_shutdown():
        plot.refresh()
        rospy.sleep(0.05)


if __name__ == '__main__':
    main()

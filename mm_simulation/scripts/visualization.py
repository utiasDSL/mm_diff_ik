#!/usr/bin/env python2
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations as tfs

from sensor_msgs.msg import JointState

from mm_msgs.msg import Obstacles
import mm_kinematics.kinematics as kinematics


class RobotPlotter(object):
    def __init__(self):
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState,
                                                self._joint_state_cb)

        self.obstacle_sub = rospy.Subscriber('/obstacles', Obstacles,
                                             self._obstacle_cb)

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

        w_p_b, w_p_e, xs, ys, zs = self._calc_arm_positions()
        x_obs, y_obs, z_obs = self._calc_obs_positions()

        self.line, = self.ax.plot(xs, ys, zs=zs, marker='o')
        self.ax.plot([0], [0], zs=[0], c='b', marker='o', label='Origin')
        self.base_pt, = self.ax.plot([w_p_b[0]], [w_p_b[1]], zs=[w_p_b[2]],
                                     c='g', marker='o', label='Base')
        self.ee_pt, = self.ax.plot([w_p_e[0]], [w_p_e[1]], zs=[w_p_e[2]],
                                   c='r', marker='o', label='EE')

        self.obs_plot, = self.ax.plot(x_obs, y_obs, zs=z_obs, marker='o', c='k')

        self.ax.legend()

    def _joint_state_cb(self, msg):
        # TODO may need a lock here
        self.q = np.array(msg.position)

    def _obstacle_cb(self, msg):
        self.obstacles = msg.obstacles

    def _calc_arm_positions(self):
        Ts = kinematics.forward_chain(self.q)
        w_T_b = Ts[3]
        w_T_e = Ts[-1]

        w_p_b = tfs.translation_from_matrix(w_T_b)
        w_p_e = tfs.translation_from_matrix(w_T_e)

        xs = [T[0,3] for T in Ts[3:]]
        ys = [T[1,3] for T in Ts[3:]]
        zs = [T[2,3] for T in Ts[3:]]

        return w_p_b, w_p_e, xs, ys, zs

    def _calc_obs_positions(self):
        x_obs = [obs.centre.x for obs in self.obstacles]
        y_obs = [obs.centre.y for obs in self.obstacles]
        z_obs = [obs.centre.z for obs in self.obstacles]

        return x_obs, y_obs, z_obs

    def refresh(self):
        ''' Update plot based on current transforms. '''
        w_p_b, w_p_e, xs, ys, zs = self._calc_arm_positions()
        x_obs, y_obs, z_obs = self._calc_obs_positions()

        self.line.set_xdata(xs)
        self.line.set_ydata(ys)
        self.line.set_3d_properties(zs)

        self.base_pt.set_xdata([w_p_b[0]])
        self.base_pt.set_ydata([w_p_b[1]])
        self.base_pt.set_3d_properties([w_p_b[2]])

        self.ee_pt.set_xdata([w_p_e[0]])
        self.ee_pt.set_ydata([w_p_e[1]])
        self.ee_pt.set_3d_properties([w_p_e[2]])

        self.obs_plot.set_xdata(x_obs)
        self.obs_plot.set_ydata(y_obs)
        self.obs_plot.set_3d_properties(z_obs)

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

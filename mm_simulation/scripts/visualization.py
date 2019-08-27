#!/usr/bin/env python2
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from sensor_msgs.msg import JointState

from mm_kinematics import ThingKinematics, R_t_from_T


class RobotPlotter(object):
    def __init__(self):
        self.kin = ThingKinematics()
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState,
                                                self._joint_state_cb)

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
        w_p_b, w_p_e, xs, ys, zs = self._calc_positions()

        self.line, = self.ax.plot(xs, ys, zs=zs, marker='o')
        self.ax.plot([0], [0], zs=[0], c='b', marker='o', label='Origin')
        self.base_pt, = self.ax.plot([w_p_b[0]], [w_p_b[1]], zs=[w_p_b[2]],
                                     c='g', marker='o', label='Base')
        self.ee_pt, = self.ax.plot([w_p_e[0]], [w_p_e[1]], zs=[w_p_e[2]],
                                   c='r', marker='o', label='EE')

        self.ax.legend()

    def _joint_state_cb(self, msg):
        # TODO may need a lock here
        self.q = np.array(msg.position)

    def _calc_positions(self):
        Ts = self.kin.calc_fk_chain(self.q)
        w_T_b = Ts[3]
        w_T_e = Ts[-1]

        _, w_p_b = R_t_from_T(w_T_b)
        _, w_p_e = R_t_from_T(w_T_e)

        xs = [T[0,3] for T in Ts[4:]]
        ys = [T[1,3] for T in Ts[4:]]
        zs = [T[2,3] for T in Ts[4:]]

        return w_p_b, w_p_e, xs, ys, zs

    def refresh(self):
        ''' Update plot based on current transforms. '''
        w_p_b, w_p_e, xs, ys, zs = self._calc_positions()

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


def main():
    rospy.init_node('mm_visualization')

    plot = RobotPlotter()
    plot.start(q0=np.zeros(9))

    while not rospy.is_shutdown():
        plot.refresh()
        rospy.sleep(0.05)


if __name__ == '__main__':
    main()

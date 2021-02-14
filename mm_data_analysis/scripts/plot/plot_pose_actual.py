#!/usr/bin/env python2
from __future__ import print_function

import sys

import rosbag
import matplotlib.pyplot as plt
import mm_data_analysis.plot as mmplt
import mm_data_analysis.util as util


def main():
    bag = rosbag.Bag(sys.argv[1])
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]

    t = util.parse_time(pose_msgs)

    # position
    px = [msg.actual.position.x for msg in pose_msgs]
    py = [msg.actual.position.y for msg in pose_msgs]
    pz = [msg.actual.position.z for msg in pose_msgs]

    # orientation
    qx = [msg.actual.orientation.x for msg in pose_msgs]
    qy = [msg.actual.orientation.y for msg in pose_msgs]
    qz = [msg.actual.orientation.z for msg in pose_msgs]

    # desired linear velocity
    vx = [msg.twist_actual.linear.x for msg in pose_msgs]
    vy = [msg.twist_actual.linear.y for msg in pose_msgs]
    vz = [msg.twist_actual.linear.z for msg in pose_msgs]

    # desired angular velocity
    wx = [msg.twist_actual.angular.x for msg in pose_msgs]
    wy = [msg.twist_actual.angular.y for msg in pose_msgs]
    wz = [msg.twist_actual.angular.z for msg in pose_msgs]

    plt.figure()
    plt.plot(t, px, 'r', label='$p_x$')
    plt.plot(t, py, 'g', label='$p_y$')
    plt.plot(t, pz, 'b', label='$p_z$')
    plt.grid()
    plt.legend()
    plt.title('End Effector Actual Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')

    plt.figure()
    plt.plot(t, qx, 'r', label='$q_x$')
    plt.plot(t, qy, 'g', label='$q_y$')
    plt.plot(t, qz, 'b', label='$q_z$')
    plt.grid()
    plt.legend()
    plt.title('End Effector Actual Orientation')
    plt.xlabel('Time (s)')
    plt.ylabel('Quaternion unit')

    plt.figure()
    plt.plot(t, vx, 'r', label='$v_x$')
    plt.plot(t, vy, 'g', label='$v_y$')
    plt.plot(t, vz, 'b', label='$v_z$')
    plt.grid()
    plt.legend()
    plt.title('End Effector Actual Linear Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')

    plt.figure()
    plt.plot(t, wx, 'r', label='$w_x$')
    plt.plot(t, wy, 'g', label='$w_y$')
    plt.plot(t, wz, 'b', label='$w_z$')
    plt.grid()
    plt.legend()
    plt.title('End Effector Actual Angular Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')

    plt.figure()
    plt.plot(t, pz, 'b', label='$p_z$')
    plt.grid()
    plt.legend()
    plt.title('z')
    plt.xlabel('Time (s)')
    plt.ylabel('z (m)')

    plt.show()


if __name__ == '__main__':
    main()

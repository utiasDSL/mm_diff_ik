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

    pds = [msg.desired.position for msg in pose_msgs]
    qds = [msg.desired.orientation for msg in pose_msgs]

    # desired position
    pdx = [p.x for p in pds]
    pdy = [p.y for p in pds]
    pdz = [p.z for p in pds]

    # desired orientation
    qdx = [q.x for q in qds]
    qdy = [q.y for q in qds]
    qdz = [q.z for q in qds]

    # desired linear velocity
    vdx = [msg.twist_desired.linear.x for msg in pose_msgs]
    vdy = [msg.twist_desired.linear.y for msg in pose_msgs]
    vdz = [msg.twist_desired.linear.z for msg in pose_msgs]

    # desired angular velocity
    wdx = [msg.twist_desired.angular.x for msg in pose_msgs]
    wdy = [msg.twist_desired.angular.y for msg in pose_msgs]
    wdz = [msg.twist_desired.angular.z for msg in pose_msgs]

    plt.figure()
    plt.plot(t, pdx, 'r', label='$p_x$')
    plt.plot(t, pdy, 'g', label='$p_y$')
    plt.plot(t, pdz, 'b', label='$p_z$')
    plt.grid()
    plt.legend()
    plt.title('End Effector Desired Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')

    plt.figure()
    plt.plot(t, qdx, 'r', label='$q_x$')
    plt.plot(t, qdy, 'g', label='$q_y$')
    plt.plot(t, qdz, 'b', label='$q_z$')
    plt.grid()
    plt.legend()
    plt.title('End Effector Desired Orientation')
    plt.xlabel('Time (s)')
    plt.ylabel('Quaternion unit')

    plt.figure()
    plt.plot(t, vdx, 'r', label='$v_x$')
    plt.plot(t, vdy, 'g', label='$v_y$')
    plt.plot(t, vdz, 'b', label='$v_z$')
    plt.grid()
    plt.legend()
    plt.title('End Effector Desired Linear Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')

    plt.figure()
    plt.plot(t, wdx, 'r', label='$w_x$')
    plt.plot(t, wdy, 'g', label='$w_y$')
    plt.plot(t, wdz, 'b', label='$w_z$')
    plt.grid()
    plt.legend()
    plt.title('End Effector Desired Angular Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')

    plt.show()



if __name__ == '__main__':
    main()

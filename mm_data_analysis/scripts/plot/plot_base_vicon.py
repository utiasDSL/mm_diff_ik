#!/usr/bin/env python2
import numpy as np
import sys
import rosbag
import mm_data_analysis.util as util
import tf.transformations as tfs
import mm_kinematics.kinematics as kinematics
import matplotlib.pyplot as plt

import IPython


def numerical_diff(p, t):
    dt = t[1:] - t[:-1]
    v = (p[1:] - p[:-1]) / dt[:, None]
    return v


def smooth(a):
    a_smooth = np.zeros_like(a)
    for i in range(a.shape[0]):
        if i == 0:
            a_smooth[i, :] = (a[i,:] + a[i+1,:]) / 2.0
        elif i == a.shape[0] - 1:
            a_smooth[i, :] = (a[i-1,:] + a[i,:]) / 2.0
        else:
            a_smooth[i, :] = (a[i-1,:] + a[i,:] + a[i+1,:]) / 3.0
    return a_smooth


def main():
    bag = rosbag.Bag(sys.argv[1])

    # parse base values
    base_vicon_msgs = [msg for _, msg, _ in bag.read_messages('/vicon/ThingBase2/ThingBase2')]
    qb = np.zeros((len(base_vicon_msgs), 3))
    for idx, msg in enumerate(base_vicon_msgs):
        quat = np.array([msg.transform.rotation.x, msg.transform.rotation.y,
                         msg.transform.rotation.z, msg.transform.rotation.w])
        rpy = tfs.euler_from_quaternion(quat)
        yaw = rpy[2]
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        qb[idx, :] = [x, y, yaw]
    tb = util.parse_time(base_vicon_msgs)

    plt.figure()
    plt.grid()
    plt.plot(tb, qb[:, 0], label='xb')
    plt.plot(tb, qb[:, 1], label='yb')
    plt.plot(tb, qb[:, 2], label='yawb')
    plt.xlabel('Time (s)')
    plt.ylabel('Base Pose')
    plt.legend()
    plt.title('Base Pose from Vicon')

    plt.show()


if __name__ == '__main__':
    main()

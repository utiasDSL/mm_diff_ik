#!/usr/bin/env python2
from __future__ import print_function

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import mm_data_analysis.util as util


def main():
    bagname = util.arg_or_most_recent('*.bag')
    print(bagname)
    bag = rosbag.Bag(bagname)

    # msg is just a Twist, so we have no header for timestamp
    msgs = [msg for _, msg, _ in bag.read_messages('/ridgeback_velocity_controller/cmd_vel')]
    vxs = np.array([msg.linear.x for msg in msgs])
    vys = np.array([msg.linear.y for msg in msgs])
    wzs = np.array([msg.angular.z for msg in msgs])

    plt.plot(vxs, label='vx')
    plt.plot(vys, label='vy')
    plt.plot(wzs, label='wz')
    plt.grid()
    plt.legend()
    plt.title('Base Velocity Commands')
    plt.xlabel('Message #')
    plt.ylabel('Velocity (m/s or rad/s)')

    plt.show()


if __name__ == '__main__':
    main()

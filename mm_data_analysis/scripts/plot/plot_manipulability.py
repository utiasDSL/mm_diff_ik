#!/usr/bin/env python2
from __future__ import print_function

import matplotlib.pyplot as plt
from mm_data_analysis import util, manipulability
from mm_kinematics import KinematicModel


def main():
    bagname = util.arg_or_most_recent('*.bag')
    model = KinematicModel()

    t, ms = manipulability.parse_bag(model, bagname)

    plt.figure()
    plt.plot(t, ms)
    plt.grid()
    plt.legend()
    plt.title('Manipulability Index vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Manipulability Index')
    plt.show()


if __name__ == '__main__':
    main()

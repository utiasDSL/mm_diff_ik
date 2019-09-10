#!/usr/bin/env python2

from __future__ import print_function

import rospy
import sys

from mm_trajectories.trajectory import launch, SpiralTrajectory


def main():
    rospy.init_node('trajectory_generator')
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30
    launch(SpiralTrajectory, duration)


if __name__ == '__main__':
    main()


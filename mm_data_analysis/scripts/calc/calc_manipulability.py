#!/usr/bin/env python2
"""Calculate average manipulability index for a given bag."""
from __future__ import print_function

import numpy as np
from mm_data_analysis import util, manipulability
from mm_kinematics import KinematicModel


def main():
    bagname = util.arg_or_most_recent("*.bag")
    model = KinematicModel()

    _, ms = manipulability.parse_bag(model, bagname)

    print("Average manipulability = {}".format(np.mean(ms)))


if __name__ == "__main__":
    main()

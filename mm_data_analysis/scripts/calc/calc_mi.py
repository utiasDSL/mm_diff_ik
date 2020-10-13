#!/usr/bin/env python2
# Calculate average MI for a given bag
from __future__ import print_function

import numpy as np
import sys
import mm_data_analysis.manipulability as manip


def main():
    t, mi, _ = manip.parse_bag(sys.argv[1])
    print('avg MI = {}'.format(np.mean(mi)))


if __name__ == '__main__':
    main()

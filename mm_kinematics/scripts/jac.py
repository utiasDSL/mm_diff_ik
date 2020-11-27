#!/usr/bin/env python2
import numpy as np
from mm_kinematics import kinematics
import IPython

np.set_printoptions(precision=3, suppress=True)


def main():
    q = np.zeros(9)
    J = kinematics.jacobian(q)
    print(J)


if __name__ == '__main__':
    main()

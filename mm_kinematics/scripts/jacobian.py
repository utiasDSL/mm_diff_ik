#!/usr/bin/env python2
import numpy as np
import scipy.linalg
from mm_kinematics import SymbolicKinematicModel
import tf.transformations as tfs
import IPython


def cross(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])


def main():
    np.set_printoptions(precision=3, suppress=True)
    model = SymbolicKinematicModel()
    q = np.array([0, 0, 0, 0.0, -2.3562, -1.5708, -2.3562, -1.5708, 1.5708])
    J = model.jacobian(q)

    IPython.embed()


if __name__ == "__main__":
    main()

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
    Ja = model.analytic_jacobian(q)

    T = model.calc_T_w_tool(q)
    Q = tfs.quaternion_from_matrix(T)  # TODO need correct rotation here
    H = np.hstack([cross(Q[:3]) + Q[3]*np.eye(3), -Q[:3, None]])
    E = scipy.linalg.block_diag(np.eye(3), 2*H)

    J2 = E.dot(Ja)

    IPython.embed()


if __name__ == "__main__":
    main()

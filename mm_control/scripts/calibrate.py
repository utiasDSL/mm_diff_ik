#!/usr/bin/env python2
import json
import sys

import numpy as np
import tf.transformations as tfs
from scipy.optimize import minimize

from mm_kinematics import KinematicModel

import IPython


def main():
    np.set_printoptions(precision=3, suppress=True)

    if len(sys.argv) < 2:
        print("configuration file path required")
        return

    model = KinematicModel()

    data_path = sys.argv[1]
    with open(data_path) as f:
        data = json.load(f)

    # TODO in practice, want to rotate by
    # C = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    # to get into the Vicon frame from the DH frame convention

    def cost(x):
        P1 = x[:7]
        P2 = x[7:14]
        # lambda1 = x[14]
        # lambda2 = x[15]

        r1, Q1 = P1[:3], P1[3:]
        r2, Q2 = P2[:3], P2[3:]

        T1 = np.array(tfs.quaternion_matrix(Q1))
        T1[:3, 3] = r1

        T2 = np.array(tfs.quaternion_matrix(Q2))
        T2[:3, 3] = r2

        J = 0
        for datum in data:
            r_ew_w_meas = np.array(datum["ee_position"]) + np.array([0.1, 0, 0])
            Q_we_meas = np.array(datum["ee_quaternion"])
            q = np.array(datum["joint_angles"])

            T_wb_nom = model.calc_T_w_base(q)
            T_we_nom = model.calc_T_w_ee(q)
            # T_we_nom[0, 3] += 0.1  # NOTE model error
            T_be_nom = np.linalg.inv(T_wb_nom).dot(T_we_nom)

            T_we_nom = T_wb_nom.dot(T1).dot(T_be_nom).dot(T2)
            r_ew_w_nom = tfs.translation_from_matrix(T_we_nom)
            Q_we_nom = tfs.quaternion_from_matrix(T_we_nom)

            pos_err = r_ew_w_meas - r_ew_w_nom
            quat_err = tfs.quaternion_multiply(Q_we_meas, Q_we_nom)

            J += pos_err.dot(pos_err) + quat_err.dot(quat_err)

        # Lagrange multipliers to enforce unit norm constraint on quaternions
        # J += lambda1 * (Q1.dot(Q1) - 1) + lambda2 * (Q2.dot(Q2) - 1)

        return J

    def approx_gradient(x, eps=1e-3):
        g = np.zeros_like(x)
        c0 = cost(x)
        for i in xrange(x.shape[0]):
            dx = np.zeros_like(x)
            dx[i] = eps
            g[i] = (cost(x + dx) - c0) / eps
        return g

    def eq_con1(x):
        """Unit norm constraint for first quaternion."""
        P1 = x[:7]
        _, Q1 = P1[:3], P1[3:]
        return Q1.dot(Q1) - 1

    def eq_con2(x):
        """Unit norm constraint for second quaternion."""
        P2 = x[7:14]
        _, Q2 = P2[:3], P2[3:]
        return Q2.dot(Q2) - 1

    x0 = np.zeros(16)
    x0[6] = 1
    x0[13] = 1

    # g = approx_gradient(x0)

    res = minimize(
        cost,
        x0,
        constraints=[{"type": "eq", "fun": eq_con1}, {"type": "eq", "fun": eq_con2}],
    )

    IPython.embed()


if __name__ == "__main__":
    main()

#!/usr/bin/env python2
"""Calibrate the robot from collected data.

Optimizes over two SE(3) transforms, one at the base and one at the end
effector, that minimize the error between the forward kinematic model and
actual EE measurements.
"""
import json
import sys

import numpy as np
import tf.transformations as tfs
from scipy.optimize import minimize

from mm_kinematics import KinematicModel

import IPython


def transforms_from_opt_var(x):
    """Convert optimization variable into two 4x4 tranform matrices."""
    P1 = x[:7]
    P2 = x[7:14]

    r1, Q1 = P1[:3], P1[3:]
    r2, Q2 = P2[:3], P2[3:]

    T1 = tfs.quaternion_matrix(Q1)
    T1[:3, 3] = r1

    T2 = tfs.quaternion_matrix(Q2)
    T2[:3, 3] = r2

    return T1, T2


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

    # error added for testing purposes
    err = np.array([0.0, 0, 0])

    def cost(x):
        T1, T2 = transforms_from_opt_var(x)

        C = 0
        for datum in data:
            r_ew_w_meas = np.array(datum["ee_position"]) + err
            Q_we_meas = np.array(datum["ee_quaternion"])
            q = np.array(datum["joint_angles"])

            T_wb_nom = model.calc_T_w_base(q)
            T_we_nom = model.calc_T_w_ee(q)
            T_be_nom = np.linalg.inv(T_wb_nom).dot(T_we_nom)

            T_we_nom = T_wb_nom.dot(T1).dot(T_be_nom).dot(T2)
            r_ew_w_nom = tfs.translation_from_matrix(T_we_nom)
            Q_we_nom = tfs.quaternion_from_matrix(T_we_nom)

            pos_err = r_ew_w_meas - r_ew_w_nom
            quat_err = tfs.quaternion_multiply(Q_we_meas, Q_we_nom)

            C += pos_err.dot(pos_err) + quat_err.dot(quat_err)

        return C

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

    x0 = np.zeros(14)
    x0[6] = 1
    x0[13] = 1

    res = minimize(
        cost,
        x0,
        constraints=[{"type": "eq", "fun": eq_con1}, {"type": "eq", "fun": eq_con2}],
    )

    if not res.success:
        print("Optimization not successful.")
        IPython.embed()
        return

    P1 = res.x[:7]
    P2 = res.x[7:14]
    r1, Q1 = P1[:3], P1[3:]
    r2, Q2 = P2[:3], P2[3:]

    if not np.isclose(Q1.dot(Q1), 1) or not np.isclose(Q2.dot(Q2), 1):
        print("Quaternion magnitude not close to 1.")
        IPython.embed()

    params = {"r1": list(r1), "Q1": list(Q1), "r2": list(r2), "Q2": list(Q2)}

    # data_path = os.path.join(pkg_path, "data", data_file_name)
    param_path = data_path.replace("calibration_data", "calibration_params")
    with open(param_path, "w") as f:
        json.dump(params, f, indent=2)

    print("Wrote params to {}.".format(param_path))


if __name__ == "__main__":
    main()

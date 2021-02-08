#!/usr/bin/env python2
import sympy as sym
import numpy as np
from mm_kinematics.symbolic import SymbolicKinematics
import IPython


def get_sub_dict(q):
    return {
        'q1': q[0],
        'q2': q[1],
        'q3': q[2],
        'q4': q[3],
        'q5': q[4],
        'q6': q[5],
    }


def mi_numeric(kin, J_sym, q):
    sub_dict = kin._sub_dict(q)
    Jn = np.array(J_sym.subs(sub_dict)).astype(np.float64)
    JJT = Jn.dot(Jn.T)
    mi = np.sqrt(np.linalg.det(JJT))
    return mi


def mi_numeric_derivative(kin, J_sym, q, dq, eps=1e-3):
    f1 = mi_numeric(kin, J_sym, q + eps * dq)
    f2 = mi_numeric(kin, J_sym, q - eps * dq)
    return (f2 - f1) / (2*eps)


def mi_analytic_derivative(kin, J_sym, dJdqi_sym, q):
    sub_dict = kin._sub_dict(q)

    Jn = np.array(J_sym.subs(sub_dict)).astype(np.float64)
    dJdqin = np.array(dJdqi_sym.subs(sub_dict)).astype(np.float64)

    JJT = Jn.dot(Jn.T)

    mi = np.sqrt(np.linalg.det(JJT))
    dJJT_dqi = dJdqin.dot(Jn.T) + Jn.dot(dJdqin.T)

    # TODO should solve the matrix system of equations rather than invert
    # directly
    ddetJJT_dqi = np.linalg.det(JJT) * np.trace(np.linalg.inv(JJT).dot(dJJT_dqi))
    return -0.5 * ddetJJT_dqi / mi


def main():
    kin = SymbolicKinematics()
    J = kin.J_sym[:, 3:]

    dJdq1 = J.diff(kin.q1)
    dJdq2 = J.diff(kin.q2)
    dJdq3 = J.diff(kin.q3)
    dJdq4 = J.diff(kin.q4)
    dJdq5 = J.diff(kin.q5)
    dJdq6 = J.diff(kin.q6)

    # q = np.array([0, 0, 0, 0, 0.5*np.pi, -0.5*np.pi, 0, 0, 0])

    q = np.array([-1.0, -1.0, 0.0, 0.0, -2.3562, -1.5708, -2.3562, -1.5708, 1.5708])
    # sub_dict = kin._sub_dict(q)
    #
    # Jn = np.array(J.subs(sub_dict)).astype(np.float64)
    # dJdq1n = np.array(dJdq1.subs(sub_dict)).astype(np.float64)
    #
    # JJT = Jn.dot(Jn.T)
    #
    # mi = np.sqrt(np.linalg.det(JJT))
    # dJJT_dq1 = dJdq1n.dot(Jn.T) + Jn.dot(dJdq1n.T)
    # ddetJJT_dq1 = np.linalg.det(JJT) * np.trace(np.linalg.inv(JJT).dot(dJJT_dq1))
    # dmi_dq1 = -0.5 * ddetJJT_dq1 / mi
    print(mi_analytic_derivative(kin, J, dJdq3, q))

    dq1 = np.array([0, 0, 0, 1, 0, 0, 0, 0, 0])
    dq3 = np.array([0, 0, 0, 0, 0, 1, 0, 0, 0])
    print(mi_numeric_derivative(kin, J, q, dq3))

    IPython.embed()


if __name__ == '__main__':
    main()

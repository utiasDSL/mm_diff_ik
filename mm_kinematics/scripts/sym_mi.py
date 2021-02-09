#!/usr/bin/env python2
import sympy as sym
import numpy as np
from mm_kinematics.symbolic import KinematicModel
import IPython


# def get_sub_dict(q):
#     return {
#         'q1': q[0],
#         'q2': q[1],
#         'q3': q[2],
#         'q4': q[3],
#         'q5': q[4],
#         'q6': q[5],
#     }


# def mi_numeric(kin, J_sym, q):
#     sub_dict = kin._sub_dict(q)
#     Jn = np.array(J_sym.subs(sub_dict)).astype(np.float64)
#     JJT = Jn.dot(Jn.T)
#     mi = np.sqrt(np.linalg.det(JJT))
#     return mi


def mi_numeric_derivative(model, q, dq, eps=1e-3):
    f1 = model.manipulability(q + eps * dq)
    f2 = model.manipulability(q - eps * dq)
    return (f2 - f1) / (2*eps)


def mi_analytic_derivative(model, dJdq_func, q):
    J = model.jacobian(q)
    dJdq = dJdq_func(q)

    JJT = J.dot(J.T)

    mi = np.sqrt(np.linalg.det(JJT))
    dJJT_dqi = dJdq.dot(J.T) + J.dot(dJdq.T)

    # TODO should solve the matrix system of equations rather than invert
    # directly
    # ddetJJT_dqi = np.linalg.det(JJT) * np.trace(np.linalg.inv(JJT).dot(dJJT_dqi))
    ddetJJT_dqi = np.linalg.det(JJT) * np.trace(np.linalg.solve(JJT, dJJT_dqi))
    return -0.5 * ddetJJT_dqi / mi


def main():
    sym.init_printing()
    model = KinematicModel()

    # home configuration
    q = np.array([-1.0, -1.0, 0.0, 0.0, -2.3562, -1.5708, -2.3562, -1.5708, 1.5708])

    # create functions to evaluate derivatives of J w.r.t. each q
    J_param = model.parameterize(model.J)
    dJdqs = [J_param.diff(qi) for qi in model.q]
    dJdqs = [sym.lambdify([model.q], dJdqi) for dJdqi in dJdqs]

    dQ = np.eye(9)

    for i in xrange(9):
        dq = dQ[i, :]
        mi_num_der = mi_numeric_derivative(model, q, dq)
        mi_ana_der = mi_analytic_derivative(model, dJdqs[i], q)
        print('Numeric derivative = {}'.format(mi_num_der))
        print('Analytic derivative = {}'.format(mi_ana_der))

    IPython.embed()

    # dJdq1 = J.diff(kin.q1)
    # dJdq2 = J.diff(kin.q2)
    # dJdq3 = J.diff(kin.q3)
    # dJdq4 = J.diff(kin.q4)
    # dJdq5 = J.diff(kin.q5)
    # dJdq6 = J.diff(kin.q6)

    # q = np.array([0, 0, 0, 0, 0.5*np.pi, -0.5*np.pi, 0, 0, 0])

    # q = np.array([-1.0, -1.0, 0.0, 0.0, -2.3562, -1.5708, -2.3562, -1.5708, 1.5708])
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
    # print(mi_analytic_derivative(kin, J, dJdq3, q))
    #
    # dq1 = np.array([0, 0, 0, 1, 0, 0, 0, 0, 0])
    # dq3 = np.array([0, 0, 0, 0, 0, 1, 0, 0, 0])
    # print(mi_numeric_derivative(kin, J, q, dq3))



if __name__ == '__main__':
    main()

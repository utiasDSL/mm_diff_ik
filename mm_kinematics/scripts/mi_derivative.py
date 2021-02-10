#!/usr/bin/env python2
import sympy as sym
import numpy as np
from mm_kinematics.symbolic import KinematicModel
import IPython


def mi_numeric_derivative(model, q, dq, eps=1e-3):
    """Numeric derivative of manipulability."""
    f1 = model.manipulability(q + eps * dq)
    f2 = model.manipulability(q - eps * dq)
    return (f2 - f1) / (2*eps)


def mi_analytic_derivative(model, dJdq_func, q):
    """Analytic derivative of manipulability."""
    J = model.jacobian(q)
    dJdq = dJdq_func(q)

    JJT = J.dot(J.T)

    mi = np.sqrt(np.linalg.det(JJT))
    dJJT_dqi = dJdq.dot(J.T) + J.dot(dJdq.T)

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


if __name__ == '__main__':
    main()

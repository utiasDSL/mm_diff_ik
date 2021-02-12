#!/usr/bin/env python2
import sympy as sym
import numpy as np
from mm_kinematics import KinematicModel
import IPython


def mi_numeric_derivative(model, q, dq, eps=1e-3):
    """Numeric derivative of manipulability."""
    f1 = model.manipulability(q - eps * dq)
    f2 = model.manipulability(q + eps * dq)
    return (f2 - f1) / (2*eps)


def mi_analytic_derivative(model, dJdq_func, q):
    """Analytic derivative of manipulability."""
    J = model.jacobian(q)[:, 3:]
    dJdq = dJdq_func(q)

    JJT = J.dot(J.T)
    m = np.sqrt(np.linalg.det(JJT))
    return m * np.trace(np.linalg.solve(JJT, J.dot(dJdq.T)))


def main():
    sym.init_printing()
    model = KinematicModel()

    # home configuration
    q = np.array([-1.0, -1.0, 0.0, 0.0, -2.3562, -1.5708, -2.3562, -1.5708, 1.5708])

    # create functions to evaluate derivatives of Ja w.r.t. each q
    Ja_param = model.parameterize(model.J[:, 3:])
    dJdqs = [Ja_param.diff(qi) for qi in model.q]
    dJdqs = [sym.lambdify([model.q], dJdqi) for dJdqi in dJdqs]

    dQ = np.eye(9)

    m_grad_a = np.zeros(9)
    m_grad_n = np.zeros(9)

    for i in xrange(9):
        dq = dQ[i, :]
        m_grad_n[i] = mi_numeric_derivative(model, q, dq)
        m_grad_a[i] = mi_analytic_derivative(model, dJdqs[i], q)

    print('Numerical gradient = {}'.format(m_grad_n))
    print('Analytic gradient = {}'.format(m_grad_a))


if __name__ == '__main__':
    main()

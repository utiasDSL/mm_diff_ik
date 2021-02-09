import re
import numpy as np
import sympy as sym

from util import dh_tf, R_t_from_T, _as_np

import IPython


def _sin_repl(m):
    return 's' + m.group(1)


def _cos_repl(m):
    return 'c' + m.group(1)


def _replace_sin_cos(s):
    s = re.sub('sin\(([a-z0-9]+)\)', _sin_repl, s)
    s = re.sub('cos\(([a-z0-9]+)\)', _cos_repl, s)
    return s


PARAM_SUB_DICT = {
    'px': 0.27,
    'py': 0.01,
    'pz': 0.653,
    'd1': 0.1273,
    'a2': -0.612,
    'a3': -0.5723,
    'd4': 0.163941,
    'd5': 0.1157,
    'd6': 0.0922,
    'd7': 0.290
}


class KinematicModel(object):
    """Symbolic kinematic model for the Thing mobile manipulator."""
    def __init__(self):
        self._calc_forward_transforms()
        self._calc_geometric_jacobian()
        self._lambdify_functions()

    def _calc_forward_transforms(self):
        """Calculate symbolic transforms from intermediate points to world."""

        # Joint variables.
        self.q = sym.symbols('xb,yb,tb,q1:7')

        # Offset from base to arm.
        px, py, pz = sym.symbols('px,py,pz')

        # Arm D-H parameters.
        d1, a2, a3, d4, d5, d6, d7 = sym.symbols('d1,a2,a3,d4,d5,d6,d7')

        self.T = [None] * 13

        # Transform from base to world.
        self.T[0] = dh_tf(sym.pi/2,  0, 0,         sym.pi/2)
        self.T[1] = dh_tf(sym.pi/2,  0, self.q[0], sym.pi/2)
        self.T[2] = dh_tf(sym.pi/2,  0, self.q[1], sym.pi/2)
        self.T[3] = dh_tf(self.q[2], 0, 0,         0)

        # Transform from arm to base.
        self.T[4] = dh_tf(0, px, pz, -sym.pi/2)
        self.T[5] = dh_tf(0, 0,  py,  sym.pi/2)

        # Transform from end effector to arm.
        self.T[6]  = dh_tf(self.q[3], 0,  d1,  sym.pi/2)
        self.T[7]  = dh_tf(self.q[4], a2, 0,   0)
        self.T[8]  = dh_tf(self.q[5], a3, 0,   0)
        self.T[9]  = dh_tf(self.q[6], 0,  d4,  sym.pi/2)
        self.T[10] = dh_tf(self.q[7], 0,  d5, -sym.pi/2)
        self.T[11] = dh_tf(self.q[8], 0,  d6,  0)
        self.T[12] = dh_tf(0,         0,  d7,  0)

        # Transforms from intermediate points to world.
        self.T0 = [sym.eye(4)]

        # Construct intermediate transforms.
        for i in xrange(13):
            self.T0.append(self.T0[i] * self.T[i])

        # self.T0[1]  = self.T0[0]  * self.T[0]
        # self.T0[2]  = self.T0[1]  * self.T[1]   # xb
        # self.T0[3]  = self.T0[2]  * self.T[2]   # yb
        # self.T0[4]  = self.T0[3]  * self.T[3]   # tb - this is w_T_b
        # self.T0[5]  = self.T0[4]  * self.T[4]
        # self.T0[6]  = self.T0[5]  * self.T[5]
        # self.T0[7]  = self.T0[6]  * self.T[6]   # q1
        # self.T0[8]  = self.T0[7]  * self.T[7]   # q2
        # self.T0[9]  = self.T0[8]  * self.T[8]   # q3
        # self.T0[10] = self.T0[9]  * self.T[9]   # q4
        # self.T0[11] = self.T0[10] * self.T[10]  # q5
        # self.T0[12] = self.T0[11] * self.T[11]  # q6
        # self.T0[13] = self.T0[12] * self.T[12]  # tool offset

    def _calc_geometric_jacobian(self):
        R0_0,  t0_0  = R_t_from_T(self.T0[0])
        R0_1,  t0_1  = R_t_from_T(self.T0[1])
        R0_2,  t0_2  = R_t_from_T(self.T0[2])
        R0_3,  t0_3  = R_t_from_T(self.T0[3])
        R0_4,  t0_4  = R_t_from_T(self.T0[4])
        R0_5,  t0_5  = R_t_from_T(self.T0[5])
        R0_6,  t0_6  = R_t_from_T(self.T0[6])
        R0_7,  t0_7  = R_t_from_T(self.T0[7])
        R0_8,  t0_8  = R_t_from_T(self.T0[8])
        R0_9,  t0_9  = R_t_from_T(self.T0[9])
        R0_10, t0_10 = R_t_from_T(self.T0[10])
        R0_11, t0_11 = R_t_from_T(self.T0[11])
        R0_12, t0_12 = R_t_from_T(self.T0[12])
        R0_13, t0_13 = R_t_from_T(self.T0[13])

        # Angular derivatives - this is the unit vector pointing along the
        # z-axis for the corresponding joint.
        k = sym.Matrix([0, 0, 1])  # Unit vector along z-axis

        z_xb = R0_2 * k
        z_yb = R0_3 * k
        z_tb = R0_4 * k

        z_q1 = R0_7  * k
        z_q2 = R0_8  * k
        z_q3 = R0_9  * k
        z_q4 = R0_10 * k
        z_q5 = R0_11 * k
        z_q6 = R0_12 * k

        # joints xb and yb are prismatic, and so cause no angular velocity.
        # TODO clean this up
        Jw = sym.Matrix.hstack(0*z_xb, 0*z_yb, z_yb, z_tb, z_q1, z_q2, z_q3, z_q4,
                               z_q5)

        # Linear derivatives
        Jvs = [sym.diff(t0_13, qi) for qi in self.q]
        Jv = sym.Matrix.hstack(*Jvs)

        # Full Jacobian
        self.J = sym.Matrix.vstack(Jv, Jw)

    def _lambdify_functions(self):
        """Create lamdified kinematic functions."""
        self.jacobian = sym.lambdify([self.q], self.J.subs(PARAM_SUB_DICT))
        self.forward = sym.lambdify([self.q], self.T0[-1].subs(PARAM_SUB_DICT))

    def parameterize(self, expr):
        """Parameterize the expression with the constant model parameters."""
        return expr.subs(PARAM_SUB_DICT)

    def manipulability(self, q):
        """Calculate manipulability index."""
        J = self.jacobian(q)
        # Ja = J[:, 3:]
        Ja = J

        m2 = np.linalg.det(Ja.dot(Ja.T))

        # handle numerical errors pushing us slightly negative
        if m2 < 0:
            m2 = 0
        m = np.sqrt(m2)
        return m

    def write_sym_jac(self, fname, fmt='c++'):
        ''' Write symbolic Jacobian out to a file. '''
        # fmt can be c++ or py

        # Generate the string version of the Jacobian
        J = np.empty((6, 9), dtype=object)
        for i in range(6):
            for j in range(9):
                s = str(self.J_sym[i,j])
                J[i, j] = _replace_sin_cos(s)

        if fmt == 'c++':
            msg = '{}({},{}) = {};\n'
        else:
            msg = '{}[{},{}] = {}\n'

        with open(fname, 'w+') as f:
            f.write('Base\n')
            for i in range(6):
                for j in range(3):
                    f.write(msg.format('Jb', i, j, J[i,j]))

            f.write('\nArm\n')
            for i in range(6):
                for j in range(6):
                    f.write(msg.format('Ja', i, j, J[i,j+3]))

    # def write_sym_orientation_jacobian(self, fname, fmt='c++'):
    #     Jn, Js, Ja = self._calc_sym_analytic_jacobian()
    #
    #     if fmt == 'c++':
    #         msg = '{}({},{}) = {};\n'
    #     else:
    #         msg = '{}[{},{}] = {}\n'
    #
    #     r, c = Jn.shape
    #     with open(fname, 'w+') as f:
    #         # Jn_str = np.empty(Jn.shape, dtype=object)
    #         f.write('Jn\n')
    #         for i in range(r):
    #             for j in range(c):
    #                 s = str(Jn[i, j])
    #                 s = _replace_sin_cos(s)
    #                 f.write(msg.format('Jn', i, j, s))
    #
    #         f.write('\nJs\n')
    #         for i in range(r):
    #             for j in range(c):
    #                 s = str(Js[i, j])
    #                 s = _replace_sin_cos(s)
    #                 f.write(msg.format('Js', i, j, s))
    #
    #         f.write('\nJa\n')
    #         for i in range(r):
    #             for j in range(c):
    #                 s = str(Ja[i, j])
    #                 s = _replace_sin_cos(s)
    #                 f.write(msg.format('Ja', i, j, s))

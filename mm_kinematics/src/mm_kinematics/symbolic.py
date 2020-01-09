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



class SymbolicKinematics(object):
    ''' Kinematics for the Thing mobile manipulator. '''
    def __init__(self):
        self._calc_sym_transforms()
        self._calc_sym_jacobian()

    def _calc_sym_transforms(self):
        ''' Calculate symbolic transforms from intermediate points to world. '''

        # Arm joints/variables.
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6 = sym.symbols('q1,q2,q3,q4,q5,q6')

        # Base joint/variables.
        self.xb, self.yb, self.tb = sym.symbols('xb,yb,tb')

        # Offset from base to arm.
        px, py, pz = sym.symbols('px,py,pz')

        # Arm D-H parameters.
        d1, a2, a3, d4, d5, d6 = sym.symbols('d1,a2,a3,d4,d5,d6')

        self.T = [None] * 12

        # Transform from base to world.
        self.T[0] = dh_tf(sym.pi/2, 0, 0,       sym.pi/2)
        self.T[1] = dh_tf(sym.pi/2, 0, self.xb, sym.pi/2)
        self.T[2] = dh_tf(sym.pi/2, 0, self.yb, sym.pi/2)
        self.T[3] = dh_tf(self.tb,  0, 0,       0)

        # Transform from arm to base.
        self.T[4] = dh_tf(0, px, pz, -sym.pi/2)
        self.T[5] = dh_tf(0, 0,  py,  sym.pi/2)

        # Transform from end effector to arm.
        self.T[6]  = dh_tf(self.q1, 0,  d1,  sym.pi/2)
        self.T[7]  = dh_tf(self.q2, a2, 0,   0)
        self.T[8]  = dh_tf(self.q3, a3, 0,   0)
        self.T[9]  = dh_tf(self.q4, 0,  d4,  sym.pi/2)
        self.T[10] = dh_tf(self.q5, 0,  d5, -sym.pi/2)
        self.T[11] = dh_tf(self.q6, 0,  d6,  0)

        # Transforms from intermediate points to world.
        self.T0 = [None] * 13

        # Construct intermediate transforms.
        self.T0[0]  = sym.eye(4)
        self.T0[1]  = self.T0[0]  * self.T[0]
        self.T0[2]  = self.T0[1]  * self.T[1]   # xb
        self.T0[3]  = self.T0[2]  * self.T[2]   # yb
        self.T0[4]  = self.T0[3]  * self.T[3]   # tb - this is w_T_b
        self.T0[5]  = self.T0[4]  * self.T[4]
        self.T0[6]  = self.T0[5]  * self.T[5]
        self.T0[7]  = self.T0[6]  * self.T[6]   # q1
        self.T0[8]  = self.T0[7]  * self.T[7]   # q2
        self.T0[9]  = self.T0[8]  * self.T[8]   # q3
        self.T0[10] = self.T0[9]  * self.T[9]   # q4
        self.T0[11] = self.T0[10] * self.T[10]  # q5
        self.T0[12] = self.T0[11] * self.T[11]  # q6

    def _calc_sym_jacobian(self):
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

        # Angular derivatives - this is the unit vector pointing along the z-axis for
        # the corresponding joint.
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
        Jw = sym.Matrix.hstack(0*z_xb, 0*z_yb, z_tb, z_q1, z_q2, z_q3, z_q4, z_q5, z_q6)

        # Linear derivatives
        dt_dxb = sym.diff(t0_12, self.xb)
        dt_dyb = sym.diff(t0_12, self.yb)
        dt_dtb = sym.diff(t0_12, self.tb)

        dt_dq1 = sym.diff(t0_12, self.q1)
        dt_dq2 = sym.diff(t0_12, self.q2)
        dt_dq3 = sym.diff(t0_12, self.q3)
        dt_dq4 = sym.diff(t0_12, self.q4)
        dt_dq5 = sym.diff(t0_12, self.q5)
        dt_dq6 = sym.diff(t0_12, self.q6)

        Jv = sym.Matrix.hstack(dt_dxb, dt_dyb, dt_dtb, dt_dq1, dt_dq2, dt_dq3, dt_dq4, dt_dq5, dt_dq6)

        # Full Jacobian
        self.J_sym = sym.Matrix.vstack(Jv, Jw)

    def _calc_sym_analytic_jacobian(self):
        # Transform from base to EE
        T = self.T0[-1]
        R, p = R_t_from_T(T)

        # Linear derivatives
        # These are the same as the geometric Jacobian
        # dp_dx = sym.diff(p, self.xb)
        # dp_dy = sym.diff(p, self.yb)
        # dp_dt = sym.diff(p, self.tb)

        # Angular derivatives
        n = R[:, 0]
        s = R[:, 1]
        a = R[:, 2]

        qs = [self.xb, self.yb, self.tb, self.q1, self.q2, self.q3, self.q4, self.q5, self.q6]

        # Calculate Jacobians for each column of the rotation matrix
        Jn = sym.Matrix.hstack(*[sym.diff(n, q) for q in qs])
        Js = sym.Matrix.hstack(*[sym.diff(s, q) for q in qs])
        Ja = sym.Matrix.hstack(*[sym.diff(a, q) for q in qs])

        return Jn, Js, Ja

    def _sub_dict(self, q):
        qb = q[:3]
        qa = q[3:]
        return {
            'xb': qb[0], 'yb': qb[1], 'tb': qb[2],
            'q1': qa[0], 'q2': qa[1], 'q3': qa[2], 'q4': qa[3], 'q5': qa[4], 'q6': qa[5],
            'px': 0.27, 'py': 0.01, 'pz': 0.653,
            'd1': 0.1273, 'a2': -0.612, 'a3': -0.5723, 'd4': 0.163941, 'd5': 0.1157, 'd6': 0.0922,
        }

    def forward_chain(self, q):
        ''' Calculate all transforms in the kinematic chain. '''
        d = self._sub_dict(q)
        return [_as_np(T0i.subs(d)) for T0i in self.T0]

    def forward(self, q):
        ''' Calculate the transform of the EE. '''
        d = self._sub_dict(q)
        return _as_np(self.T0[-1].subs(d))

    def jacobian(self, q):
        ''' Calculate the Jacobian. '''
        d = self._sub_dict(q)
        return _as_np(self.J_sym.subs(d))

    def manipulability(self, q):
        d = self._sub_dict(q)
        J = _as_np(self.J_sym.subs(d))
        Ja = J[:,3:]

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

    def write_sym_orientation_jacobian(self, fname, fmt='c++'):
        Jn, Js, Ja = self._calc_sym_analytic_jacobian()

        if fmt == 'c++':
            msg = '{}({},{}) = {};\n'
        else:
            msg = '{}[{},{}] = {}\n'

        r, c = Jn.shape
        with open(fname, 'w+') as f:
            # Jn_str = np.empty(Jn.shape, dtype=object)
            f.write('Jn\n')
            for i in range(r):
                for j in range(c):
                    s = str(Jn[i, j])
                    s = _replace_sin_cos(s)
                    f.write(msg.format('Jn', i, j, s))

            f.write('\nJs\n')
            for i in range(r):
                for j in range(c):
                    s = str(Js[i, j])
                    s = _replace_sin_cos(s)
                    f.write(msg.format('Js', i, j, s))

            f.write('\nJa\n')
            for i in range(r):
                for j in range(c):
                    s = str(Ja[i, j])
                    s = _replace_sin_cos(s)
                    f.write(msg.format('Ja', i, j, s))

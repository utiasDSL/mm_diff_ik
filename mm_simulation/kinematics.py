import sympy as sym
import re

import numpy as np


def dh_tf(q, a, d, alpha):
    ''' Transformation matrix from D-H parameters. '''
    return sym.Matrix([
        [sym.cos(q), -sym.sin(q)*sym.cos(alpha),  sym.sin(q)*sym.sin(alpha), a*sym.cos(q)],
        [sym.sin(q),  sym.cos(q)*sym.cos(alpha), -sym.cos(q)*sym.sin(alpha), a*sym.sin(q)],
        [0,           sym.sin(alpha),             sym.cos(alpha),            d],
        [0,           0,                          0,                         1]])


def R_t_from_T(T):
    ''' Extract rotation and translation from transform. '''
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    return R, t


class ThingKinematics(object):
    def __init__(self):
        self._calc_sym_transforms()
        self._calc_sym_jacobian()

    def _calc_sym_transforms(self):
        ''' Calculate symbolic transforms from intermediate points to world. '''
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6 = sym.symbols('q1,q2,q3,q4,q5,q6')
        self.xb, self.yb, self.tb = sym.symbols('xb,yb,tb')

        self.T = [None] * 12

        # Transform from base to world.
        self.T[0] = dh_tf(sym.pi/2, 0, 0,  sym.pi/2)
        self.T[1] = dh_tf(sym.pi/2, 0, self.xb, sym.pi/2)
        self.T[2] = dh_tf(sym.pi/2, 0, self.yb, sym.pi/2)
        self.T[3] = dh_tf(self.tb,       0, 0,  0)

        # Transform from arm to base.
        self.T[4] = dh_tf(0, 0.27, 0.653, -sym.pi/2)
        self.T[5] = dh_tf(0, 0,    0.01,   sym.pi/2)

        # Transform from end effector to arm.
        self.T[6]  = dh_tf(self.q1,  0,      0.1273,   sym.pi/2)
        self.T[7]  = dh_tf(self.q2, -0.612,  0,        0)
        self.T[8]  = dh_tf(self.q3, -0.5723, 0,        0)
        self.T[9]  = dh_tf(self.q4,  0,      0.163941, sym.pi/2)
        self.T[10] = dh_tf(self.q5,  0,      0.1157,  -sym.pi/2)
        self.T[11] = dh_tf(self.q6,  0,      0.0922,   0)

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

    def _sub_dict(self, qb, qa):
        return {
            'xb': qb[0], 'yb': qb[1], 'tb': qb[2],
            'q1': qa[0], 'q2': qa[1], 'q3': qa[2], 'q4': qa[3], 'q5': qa[4], 'q6': qa[5]
        }

    def _as_np(self, M):
        return np.array(M).astype(np.float64)

    def calc_fk_chain(self, qb, qa):
        ''' Calculate all transforms in the kinematic chain. '''
        d = self._sub_dict(qb, qa)
        return [self._as_np(T0i.subs(d)) for T0i in self.T0]

    def calc_fk(self, qb, qa):
        ''' Calculate the transform of the EE. '''
        d = self._sub_dict(qb, qa)
        return self._as_np(self.T0[-1].subs(d))

    def calc_jac(self, qb, qa):
        ''' Calculate the Jacobian. '''
        d = self._sub_dict(qb, qa)
        return self._as_np(self.J_sym.subs(d))

    def write_sym_jac(self, fname):
        ''' Write symbolic Jacobian out to a file. '''
        def sin_repl(m):
            return 's' + m.group(1)

        def cos_repl(m):
            return 'c' + m.group(1)

        with open(fname, 'w+') as f:
            for i in range(6):
                for j in range(9):
                    Jijs = str(self.J_sym[i,j])
                    Jijs = re.sub('sin\(([a-z0-9]+)\)', sin_repl, Jijs)
                    Jijs = re.sub('cos\(([a-z0-9]+)\)', cos_repl, Jijs)
                    f.write(Jijs + '\n')


def main():
    kin = ThingKinematics()
    qb = np.zeros(3)
    qa = np.zeros(6)
    q = np.zeros(9)
    eps = 1e-5

    J = kin.calc_jac(q[:3], q[3:])
    Ts = kin.calc_fk_chain(q[:3], q[3:])

    print(Ts[3])
    print(Ts[5])
    print(Ts[11])


    # for i in range(9):
    #     epsv = np.zeros(9)
    #     epsv[i] = eps
    #
    #     q1 = q + epsv
    #     q2 = q - epsv
    #
    #     T1 = kin.calc_fk(q1[:3], q1[3:])
    #     T2 = kin.calc_fk(q2[:3], q2[3:])
    #
    #     approx = (T1[:3,3] - T2[:3,3]) / (2*eps)
    #     real = J[:3,i]
    #     print('{} << {}'.format(approx, real))


if __name__ == '__main__':
    main()

import numpy as np
import sympy as sym

from util import dh_tf, R_t_from_T


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
        z0 = sym.Matrix([0, 0, 1])  # Unit vector along z-axis

        # axis for each joint's angular velocity is the z-axis of the previous
        # transform
        z_tb = R0_3 * z0
        z_q1 = R0_6 * z0
        z_q2 = R0_7 * z0
        z_q3 = R0_8 * z0
        z_q4 = R0_9 * z0
        z_q5 = R0_10 * z0
        z_q6 = R0_11 * z0

        # joints xb and yb are prismatic, and so cause no angular velocity.
        Jw = sym.Matrix.hstack(sym.zeros(3, 2), z_tb, z_q1, z_q2, z_q3, z_q4,
                               z_q5, z_q6)

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

"""Symbolic Python kinematics for the Thing.

This is intended to be the "source of truth" for kinematics for the Thing. This
module generates to forward and differential kinematics used by other Python
code, and generates the differential kinematics (i.e., Jacobians) for C++.
"""
import numpy as np
import sympy as sym
from sympy.algebras.quaternion import Quaternion

from mm_kinematics.util import R_t_from_T


JOINT_NAMES = ["xb", "yb", "tb", "q1", "q2", "q3", "q4", "q5", "q6"]

PARAM_SUB_DICT = {
    "px": 0.27,
    "py": 0.01,
    "pz": 0.653,
    "d1": 0.1273,
    "a2": -0.612,
    "a3": -0.5723,
    "d4": 0.163941,
    "d5": 0.1157,
    "d6": 0.0922,
    "d7": 0.290,
}


def symbolic_dh_transform(q, a, d, alpha):
    """Constuct a symbolic transformation matrix from D-H parameters."""
    return sym.Matrix(
        [
            [
                sym.cos(q),
                -sym.sin(q) * sym.cos(alpha),
                sym.sin(q) * sym.sin(alpha),
                a * sym.cos(q),
            ],
            [
                sym.sin(q),
                sym.cos(q) * sym.cos(alpha),
                -sym.cos(q) * sym.sin(alpha),
                a * sym.sin(q),
            ],
            [0, sym.sin(alpha), sym.cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )


def symbolic_transform(C=sym.Matrix.eye(3), r=sym.Matrix.zeros(3, 1)):
    """Construct a symbolic transformation matrix.

    C is the rotation matrix (defaults to identity; i.e., no rotation),
    r is the translation vector (defaults to zero)
    """
    last_row = sym.Matrix([[0, 0, 0, 1]])
    T = sym.Matrix.vstack(sym.Matrix.hstack(C, r), last_row)
    return T


# transform from palm to the EE (just an offset added by the gripper)
T_ee_palm = symbolic_dh_transform(0, 0, 0.2, 0)

# transform from force/torque sensor frame to EE frame
T_ee_ft = symbolic_transform(r=sym.Matrix([0.02, 0, 0]))


def skew(v):
    return sym.Matrix([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def cross(a, b):
    """Cross product between sympy vectors a and b."""
    return skew(a) * b


class SymbolicKinematicModel(object):
    """Symbolic kinematic model for the Thing mobile manipulator."""

    def __init__(self):
        self._calc_forward_transforms()
        self._calc_geometric_jacobian()
        # self._calc_analytic_jacobian_quat()  # this is quite slow
        self._lambdify_functions()

    def _calc_forward_transforms(self):
        """Calculate symbolic transforms from intermediate points to world."""

        # Joint variables.
        self.q = sym.symbols(JOINT_NAMES)

        # Offset from base to arm.
        px, py, pz = sym.symbols("px,py,pz")

        # Arm D-H parameters.
        d1, a2, a3, d4, d5, d6, d7 = sym.symbols("d1,a2,a3,d4,d5,d6,d7")

        self.T = [None] * 13

        # Transform from base to world.
        self.T[0] = symbolic_dh_transform(sym.pi / 2, 0, 0, sym.pi / 2)
        self.T[1] = symbolic_dh_transform(sym.pi / 2, 0, self.q[0], sym.pi / 2)
        self.T[2] = symbolic_dh_transform(sym.pi / 2, 0, self.q[1], sym.pi / 2)
        self.T[3] = symbolic_dh_transform(self.q[2], 0, 0, 0)

        # Transform from arm to base.
        self.T[4] = symbolic_dh_transform(0, px, pz, -sym.pi / 2)
        self.T[5] = symbolic_dh_transform(0, 0, py, sym.pi / 2)

        # Transform from end effector to arm.
        self.T[6] = symbolic_dh_transform(self.q[3], 0, d1, sym.pi / 2)
        self.T[7] = symbolic_dh_transform(self.q[4], a2, 0, 0)
        self.T[8] = symbolic_dh_transform(self.q[5], a3, 0, 0)
        self.T[9] = symbolic_dh_transform(self.q[6], 0, d4, sym.pi / 2)
        self.T[10] = symbolic_dh_transform(self.q[7], 0, d5, -sym.pi / 2)
        self.T[11] = symbolic_dh_transform(self.q[8], 0, d6, 0)
        self.T[12] = symbolic_dh_transform(0, 0, d7, 0)

        # Transforms from intermediate points to world.
        self.T0 = [sym.eye(4)]

        # Construct intermediate transforms.
        for i in range(13):
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
        # TODO refactor this...
        R0_0, t0_0 = R_t_from_T(self.T0[0])
        R0_1, t0_1 = R_t_from_T(self.T0[1])
        R0_2, t0_2 = R_t_from_T(self.T0[2])
        R0_3, t0_3 = R_t_from_T(self.T0[3])
        R0_4, t0_4 = R_t_from_T(self.T0[4])
        R0_5, t0_5 = R_t_from_T(self.T0[5])
        R0_6, t0_6 = R_t_from_T(self.T0[6])
        R0_7, t0_7 = R_t_from_T(self.T0[7])
        R0_8, t0_8 = R_t_from_T(self.T0[8])
        R0_9, t0_9 = R_t_from_T(self.T0[9])
        R0_10, t0_10 = R_t_from_T(self.T0[10])
        R0_11, t0_11 = R_t_from_T(self.T0[11])
        R0_12, t0_12 = R_t_from_T(self.T0[12])
        R0_13, t0_13 = R_t_from_T(self.T0[13])

        # Angular derivatives - this is the unit vector pointing along the
        # z-axis for the corresponding joint.
        z0 = sym.Matrix([0, 0, 1])  # Unit vector along z-axis

        # axis for each joint's angular velocity is the z-axis of the previous
        # transform
        z_xb = R0_1 * z0
        z_yb = R0_2 * z0
        z_tb = R0_3 * z0
        z_q1 = R0_6 * z0
        z_q2 = R0_7 * z0
        z_q3 = R0_8 * z0
        z_q4 = R0_9 * z0
        z_q5 = R0_10 * z0
        z_q6 = R0_11 * z0

        p_tb = t0_3
        p_q1 = t0_6
        p_q2 = t0_7
        p_q3 = t0_8
        p_q4 = t0_9
        p_q5 = t0_10
        p_q6 = t0_11

        # joints xb and yb are prismatic, and so cause no angular velocity.
        Jw = sym.Matrix.hstack(
            sym.zeros(3, 2), z_tb, z_q1, z_q2, z_q3, z_q4, z_q5, z_q6
        )

        pe = t0_13
        Jps = [
            z_xb,
            z_yb,
            cross(z_tb, pe - p_tb),
            cross(z_q1, pe - p_q1),
            cross(z_q2, pe - p_q2),
            cross(z_q3, pe - p_q3),
            cross(z_q4, pe - p_q4),
            cross(z_q5, pe - p_q5),
            cross(z_q6, pe - p_q6)
        ]
        self.Jp = sym.Matrix.hstack(*Jps)

        # Linear derivatives
        Jvs = [sym.diff(t0_13, qi) for qi in self.q]
        Jv = sym.Matrix.hstack(*Jvs)

        # Full Jacobian
        self.J = sym.Matrix.vstack(Jv, Jw)

    def _calc_analytic_jacobian_quat(self):
        """Calculate analytic Jacobian with rotation parameterized as quaternion."""
        T = self.T0[13]

        # TODO probably have to rotate the quaternion here too to get out of DH
        # convention
        Q = Quaternion.from_rotation_matrix(T[:3, :3])
        r = T[:3, 3]

        Jps = [sym.diff(r, q) for q in self.q]
        Jp = sym.Matrix.hstack(*Jps)

        # TODO maybe diff the quaternions then construct afterward
        Qvec = sym.Matrix([Q.b, Q.c, Q.d, Q.a])  # xyzw
        # import IPython; IPython.embed()
        Jos = [sym.diff(Qvec, q) for q in self.q]
        Jo = sym.Matrix.hstack(*Jos)

        self.Ja = sym.Matrix.vstack(Jp, Jo)

    def _lambdify_functions(self):
        """Create lamdified kinematic functions."""
        # Geometric Jacobian
        self.jacobian = sym.lambdify([self.q], self.J.subs(PARAM_SUB_DICT))

        self.Jp_func = sym.lambdify([self.q], self.Jp.subs(PARAM_SUB_DICT))

        # Analytic Jacobian
        # self.analytic_jacobian = sym.lambdify([self.q], self.Ja.subs(PARAM_SUB_DICT))

        T_w_base = self.T0[4].subs(PARAM_SUB_DICT)
        T_w_ee = self.T0[-2].subs(PARAM_SUB_DICT)
        T_w_tool = self.T0[-1].subs(PARAM_SUB_DICT)
        T_w_palm = T_w_ee * T_ee_palm
        T_w_ft = T_w_ee * T_ee_ft

        # forward transform for tool frame
        self.calc_T_w_tool = sym.lambdify([self.q], T_w_tool)

        # forward transform for EE frame
        self.calc_T_w_ee = sym.lambdify([self.q], T_w_ee)

        # forward transform for palm frame: this is the palm of the gripper,
        # which adds an offset to the EE
        self.calc_T_w_palm = sym.lambdify([self.q], T_w_palm)

        # forward transform for FT frame
        self.calc_T_w_ft = sym.lambdify([self.q], T_w_ft)

        # forward transform to base frame
        self.calc_T_w_base = sym.lambdify([self.q], T_w_base)

        # full forward kinematic chain
        chain = []
        for T in self.T0:
            chain.append(T.subs(PARAM_SUB_DICT))
        self.calc_chain = sym.lambdify([self.q], chain)

    def parameterize(self, expr):
        """Parameterize the expression with the constant model parameters."""
        return expr.subs(PARAM_SUB_DICT)

    def manipulability(self, q):
        """Calculate manipulability index."""
        J = self.jacobian(q)
        Ja = J[:, 3:]

        m2 = np.linalg.det(Ja.dot(Ja.T))

        # handle numerical errors pushing us slightly negative
        if m2 < 0:
            m2 = 0
        m = np.sqrt(m2)
        return m

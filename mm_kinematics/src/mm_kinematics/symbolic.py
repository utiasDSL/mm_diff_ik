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
        self.T0 = [self.T[0]]

        # Construct intermediate transforms.
        for i in range(1, 13):
            self.T0.append(self.T0[i - 1] * self.T[i])

    def _calc_geometric_jacobian(self):
        # TODO refactor this...
        R0, t0 = R_t_from_T(self.T0[0])
        R1, t1 = R_t_from_T(self.T0[1])
        R2, t2 = R_t_from_T(self.T0[2])
        R5, t5 = R_t_from_T(self.T0[5])
        R6, t6 = R_t_from_T(self.T0[6])
        R7, t7 = R_t_from_T(self.T0[7])
        R8, t8 = R_t_from_T(self.T0[8])
        R9, t9 = R_t_from_T(self.T0[9])
        R10, t10 = R_t_from_T(self.T0[10])
        _, p_ee = R_t_from_T(self.T0[-1])

        # Angular derivatives - this is the unit vector pointing along the
        # z-axis for the corresponding joint.
        z = sym.Matrix([0, 0, 1])  # Unit vector along z-axis

        # axis for each joint's angular velocity is the z-axis of the previous
        # transform
        # TODO this may be wrong (see implementation for OCS2 controller)
        z0 = R0 * z
        z1 = R1 * z
        z2 = R2 * z
        z5 = R5 * z
        z6 = R6 * z
        z7 = R7 * z
        z8 = R8 * z
        z9 = R9 * z
        z10 = R10 * z

        p_tb = t0_3
        p_q1 = t0_6
        p_q2 = t0_7
        p_q3 = t0_8
        p_q4 = t0_9
        p_q5 = t0_10
        p_q6 = t0_11

        # joints xb and yb are prismatic, and so cause no angular velocity.
        Jo = sym.Matrix.hstack(sym.zeros(3, 2), z2, z5, z6, z7, z8, z9, z10)
        Jp = sym.Matrix.hstack(
            z0,
            z1,
            z2.cross(p_ee - t2),
            z5.cross(p_ee - t5),
            z6.cross(p_ee - t6),
            z7.cross(p_ee - t7),
            z8.cross(p_ee - t8),
            z9.cross(p_ee - t9),
            z10.cross(p_ee - t10)
        )

        # Linear derivatives
        # Jps = [sym.diff(t0_13, qi) for qi in self.q]
        # Jp = sym.Matrix.hstack(*Jvs)

        # Full Jacobian
        self.J = sym.Matrix.vstack(Jp, Jo)

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

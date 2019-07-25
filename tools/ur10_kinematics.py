import numpy as np
import sympy as sym
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re
import quadprog

import IPython


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


sym.var('q1,q2,q3,q4,q5,q6')
sym.var('xb,yb,tb')
sym.var('dx,dy,dz')


# Transform from base to world.
T1 = dh_tf(sym.pi/2, 0, 0,  sym.pi/2)
T2 = dh_tf(sym.pi/2, 0, xb, sym.pi/2)
T3 = dh_tf(sym.pi/2, 0, yb, sym.pi/2)
T4 = dh_tf(tb,       0, 0,  0)
w_T_b = T1 * T2 * T3 * T4

# Transform from arm to base.
T5 = dh_tf(0, 0.27, 0.653, -sym.pi/2)
T6 = dh_tf(0, 0,    0.01,   sym.pi/2)
b_T_a = T5 * T6

# Transform from end effector to arm.
T7  = dh_tf(q1,  0,      0.1273,   sym.pi/2)
T8  = dh_tf(q2, -0.612,  0,        0)
T9  = dh_tf(q3, -0.5723, 0,        0)
T10 = dh_tf(q4,  0,      0.163941, sym.pi/2)
T11 = dh_tf(q5,  0,      0.1157,  -sym.pi/2)
T12 = dh_tf(q6,  0,      0.0922,   0)
a_T_e = T7 * T8 * T9 * T10 * T11 * T12

w_T_e = w_T_b * b_T_a * a_T_e

# Construct intermediate transforms.
T0_0  = sym.eye(4)
T0_1  = T0_0  * T1
T0_2  = T0_1  * T2   # xb
T0_3  = T0_2  * T3   # yb
T0_4  = T0_3  * T4   # tb - this is w_T_b
T0_5  = T0_4  * T5
T0_6  = T0_5  * T6
T0_7  = T0_6  * T7   # q1
T0_8  = T0_7  * T8   # q2
T0_9  = T0_8  * T9   # q3
T0_10 = T0_9  * T10  # q4
T0_11 = T0_10 * T11  # q5
T0_12 = T0_11 * T12  # q6

R0_0,  t0_0  = R_t_from_T(T0_0)
R0_1,  t0_1  = R_t_from_T(T0_1)
R0_2,  t0_2  = R_t_from_T(T0_2)
R0_3,  t0_3  = R_t_from_T(T0_3)
R0_4,  t0_4  = R_t_from_T(T0_4)
R0_5,  t0_5  = R_t_from_T(T0_5)
R0_6,  t0_6  = R_t_from_T(T0_6)
R0_7,  t0_7  = R_t_from_T(T0_7)
R0_8,  t0_8  = R_t_from_T(T0_8)
R0_9,  t0_9  = R_t_from_T(T0_9)
R0_10, t0_10 = R_t_from_T(T0_10)
R0_11, t0_11 = R_t_from_T(T0_11)
R0_12, t0_12 = R_t_from_T(T0_12)

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
dt_dxb = sym.diff(t0_12, xb)
dt_dyb = sym.diff(t0_12, yb)
dt_dtb = sym.diff(t0_12, tb)

dt_dq1 = sym.diff(t0_12, q1)
dt_dq2 = sym.diff(t0_12, q2)
dt_dq3 = sym.diff(t0_12, q3)
dt_dq4 = sym.diff(t0_12, q4)
dt_dq5 = sym.diff(t0_12, q5)
dt_dq6 = sym.diff(t0_12, q6)

Jv = sym.Matrix.hstack(dt_dxb, dt_dyb, dt_dtb, dt_dq1, dt_dq2, dt_dq3, dt_dq4, dt_dq5, dt_dq6)

# Full Jacobian
J = sym.Matrix.vstack(Jv, Jw)


def sub_dict(qb, qa):
    return {
        'xb': qb[0], 'yb': qb[1], 'tb': qb[2],
        'q1': qa[0], 'q2': qa[1], 'q3': qa[2], 'q4': qa[3], 'q5': qa[4], 'q6': qa[5]
    }


def calc_fk_chain(qb, qa):
    ''' Calculate all transforms in the kinematic chain. '''
    Ts = [T0_1, T0_2, T0_3, T0_4, T0_5, T0_6, T0_7, T0_8, T0_9, T0_10, T0_11, T0_12]
    d = sub_dict(qb, qa)
    return [T.subs(d) for T in Ts]


def calc_fk(qb, qa):
    ''' Calculate the transform of the EE. '''
    return w_T_e.subs(sub_dict(qb, qa))


def calc_J(qb, qa):
    ''' Calculate the Jacobian. '''
    return J.subs(sub_dict(qb, qa))


qb = [0, 0, 0]
qa = [0, 0, 0, 0, 0, 0]

Ts = calc_fk_chain(qb, qa)
w_T_b_sub = Ts[3]
w_T_e_sub = Ts[-1]

w_R_b, w_p_b = R_t_from_T(w_T_b_sub)
w_R_e, w_p_e = R_t_from_T(w_T_e_sub)

xs = [T[0,3] for T in Ts]
ys = [T[1,3] for T in Ts]
zs = [T[2,3] for T in Ts]

J_sub = calc_J(qb, qa)
dq = sym.Matrix([0, 0, 1, 0, 0, 0, 0, 0, 0])


def write_jacobian(fname):
    ''' Write symbolic Jacobian out to a file. '''
    def sin_repl(m):
        return 's' + m.group(1)

    def cos_repl(m):
        return 'c' + m.group(1)

    with open(fname, 'w+') as f:
        for i in range(6):
            for j in range(9):
                Jijs = str(J[i,j])
                Jijs = re.sub('sin\(([a-z0-9]+)\)', sin_repl, Jijs)
                Jijs = re.sub('cos\(([a-z0-9]+)\)', cos_repl, Jijs)
                f.write(Jijs + '\n')


def test_optimization():
    ''' Test the optimization problem. '''
    G = np.eye(9)
    a = np.zeros(9)

    # note that the formulation uses C.T x >= b
    C = np.array(J_sub.T).astype(np.float64)
    b = np.array([1, 1, 0, 0, 0, 0]).astype(np.float64)

    # 6 equality constraints from the Jacobian, followed by joint velocity
    # constraints
    meq = 6

    # min 1/2 x.T G x - a.T x
    # s.t C.T x >= b
    res = quadprog.solve_qp(G, a, C, b, meq)
    IPython.embed()

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(xs, ys, zs=zs, marker='o')
# ax.scatter([0], [0], zs=[0], c='b', label='Origin')
# ax.scatter([w_p_b[0]], [w_p_b[1]], zs=[w_p_b[2]], c='g', label='Base')
# ax.scatter([w_p_e[0]], [w_p_e[1]], zs=[w_p_e[2]], c='r', label='EE')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_xlim([-2, 2])
# ax.set_ylim([-2, 2])
# ax.set_zlim([0, 2])
# ax.legend()
# plt.show()

test_optimization()

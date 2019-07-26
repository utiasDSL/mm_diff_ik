import numpy as np
import sympy as sym
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import re
import quadprog
import time

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


class ThingKinematics(object):
    def __init__(self):
        self._calc_sym_transforms()
        self._calc_sym_jacobian()

    def _calc_sym_transforms(self):
        ''' Calculate symbolic transforms from intermediate points to world. '''
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6 = sym.symbols('q1,q2,q3,q4,q5,q6')
        self.xb, self.yb, self.tb = sym.symbols('xb,yb,tb')

        # Transform from base to world.
        T1 = dh_tf(sym.pi/2, 0, 0,  sym.pi/2)
        T2 = dh_tf(sym.pi/2, 0, self.xb, sym.pi/2)
        T3 = dh_tf(sym.pi/2, 0, self.yb, sym.pi/2)
        T4 = dh_tf(self.tb,       0, 0,  0)

        # Transform from arm to base.
        T5 = dh_tf(0, 0.27, 0.653, -sym.pi/2)
        T6 = dh_tf(0, 0,    0.01,   sym.pi/2)

        # Transform from end effector to arm.
        T7  = dh_tf(self.q1,  0,      0.1273,   sym.pi/2)
        T8  = dh_tf(self.q2, -0.612,  0,        0)
        T9  = dh_tf(self.q3, -0.5723, 0,        0)
        T10 = dh_tf(self.q4,  0,      0.163941, sym.pi/2)
        T11 = dh_tf(self.q5,  0,      0.1157,  -sym.pi/2)
        T12 = dh_tf(self.q6,  0,      0.0922,   0)

        # Transforms from intermediate points to world.
        self.T0 = [None] * 13

        # Construct intermediate transforms.
        self.T0[0]  = sym.eye(4)
        self.T0[1]  = self.T0[0]  * T1
        self.T0[2]  = self.T0[1]  * T2   # xb
        self.T0[3]  = self.T0[2]  * T3   # yb
        self.T0[4]  = self.T0[3]  * T4   # tb - this is w_T_b
        self.T0[5]  = self.T0[4]  * T5
        self.T0[6]  = self.T0[5]  * T6
        self.T0[7]  = self.T0[6]  * T7   # q1
        self.T0[8]  = self.T0[7]  * T8   # q2
        self.T0[9]  = self.T0[8]  * T9   # q3
        self.T0[10] = self.T0[9]  * T10  # q4
        self.T0[11] = self.T0[10] * T11  # q5
        self.T0[12] = self.T0[11] * T12  # q6

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

    def calc_fk_chain(self, qb, qa):
        ''' Calculate all transforms in the kinematic chain. '''
        d = self._sub_dict(qb, qa)
        return [T0i.subs(d) for T0i in self.T0]

    def calc_fk(self, qb, qa):
        ''' Calculate the transform of the EE. '''
        return self.T0[-1].subs(self._sub_dict(qb, qa))

    def calc_jac(self, qb, qa):
        ''' Calculate the Jacobian. '''
        return self.J_sym.subs(self._sub_dict(qb, qa))

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


class RobotVisualization(object):
    def __init__(self, q):
        # maintain an internal robot state
        self.q = q
        self.dq = dq


        # self.animation = animation.FuncAnimation(
        #     self.fig, self._animate, interval=100, blit=False)

    def _step(self, i):
        self.q += dt * self.dq
        pass


kin = ThingKinematics()

qb = [0, 0, 0]
qa = [0, 0, 0, 0, 0, 0]

Ts = kin.calc_fk_chain(qb, qa)

J = kin.calc_jac(qb, qa)
dq = sym.Matrix([0, 0, 1, 0, 0, 0, 0, 0, 0])


class RobotPlotter(object):
    def __init__(self):
        pass

    def start(self, Ts):
        ''' Launch the plot. '''
        plt.ion()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([0, 2])

        w_p_b, w_p_e, xs, ys, zs = self._calc_data(Ts)

        self.line, = self.ax.plot(xs, ys, zs=zs, marker='o')
        self.ax.plot([0], [0], zs=[0], c='b', marker='o', label='Origin')
        self.base_pt, = self.ax.plot([w_p_b[0]], [w_p_b[1]], zs=[w_p_b[2]],
                                     c='g', marker='o', label='Base')
        self.ee_pt, = self.ax.plot([w_p_e[0]], [w_p_e[1]], zs=[w_p_e[2]],
                                   c='r', marker='o', label='EE')

        self.ax.legend()

    def _calc_data(self, Ts):
        w_T_b = Ts[3]
        w_T_e = Ts[-1]

        _, w_p_b = R_t_from_T(w_T_b)
        _, w_p_e = R_t_from_T(w_T_e)

        xs = [T[0,3] for T in Ts[4:]]
        ys = [T[1,3] for T in Ts[4:]]
        zs = [T[2,3] for T in Ts[4:]]

        return w_p_b, w_p_e, xs, ys, zs

    def update(self, Ts):
        ''' Update plot based on new transforms Ts. '''
        w_p_b, w_p_e, xs, ys, zs = self._calc_data(Ts)

        self.line.set_xdata(xs)
        self.line.set_ydata(ys)
        self.line.set_3d_properties(zs)

        self.base_pt.set_xdata([w_p_b[0]])
        self.base_pt.set_ydata([w_p_b[1]])
        self.base_pt.set_3d_properties([w_p_b[2]])

        self.ee_pt.set_xdata([w_p_e[0]])
        self.ee_pt.set_ydata([w_p_e[1]])
        self.ee_pt.set_3d_properties([w_p_e[2]])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


plot = RobotPlotter()
plot.start(Ts)

i = 0
q = np.array([0]*9)
while True:
    Ts = kin.calc_fk_chain(q[:3], q[3:])
    plot.update(Ts)

    time.sleep(0.1)

    q[i] = 0
    i = (i + 1) % 9
    q[i] = 1

    # IPython.embed()







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


# test_optimization()

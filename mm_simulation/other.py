import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import quadprog
import time


# kin = ThingKinematics()
#
# qb = [0, 0, 0]
# qa = [0, 0, 0, 0, 0, 0]
#
# Ts = kin.calc_fk_chain(qb, qa)
#
# J = kin.calc_jac(qb, qa)
# dq = sym.Matrix([0, 0, 1, 0, 0, 0, 0, 0, 0])
#
# plot = RobotPlotter()
# plot.start(Ts)
#
# i = 0
# q = np.array([0]*9)
# while True:
#     Ts = kin.calc_fk_chain(q[:3], q[3:])
#     plot.update(Ts)
#
#     time.sleep(0.1)
#
#     q[i] = 0
#     i = (i + 1) % 9
#     q[i] = 1
#
#     # IPython.embed()


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

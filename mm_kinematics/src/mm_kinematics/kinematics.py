import numpy as np

import IPython


px = 0.27
py = 0.01
pz = 0.653

# Arm D-H params
d1 = 0.1273
a2 = -0.612
a3 = -0.5723
d4 = 0.163941
d5 = 0.1157
d6 = 0.0922


def _dh_tf(q, a, d, alpha):
    ''' Numerical transformation matrix from D-H parameters. '''
    return np.array([
        [np.cos(q), -np.sin(q)*np.cos(alpha),  np.sin(q)*np.sin(alpha), a*np.cos(q)],
        [np.sin(q),  np.cos(q)*np.cos(alpha), -np.cos(q)*np.sin(alpha), a*np.sin(q)],
        [0,          np.sin(alpha),            np.cos(alpha),           d],
        [0,          0,                        0,                       1]])


# Some FK transforms are constant
T0 = _dh_tf(np.pi/2, 0, 0, np.pi/2)
T4 = _dh_tf(0, px, pz, -np.pi/2)
T5 = _dh_tf(0, 0,  py,  np.pi/2)


def _calc_chain(q):
    Ts = [None] * 12

    # Transform from base to world.
    Ts[0] = T0
    Ts[1] = _dh_tf(np.pi/2, 0, q[0], np.pi/2)
    Ts[2] = _dh_tf(np.pi/2, 0, q[1], np.pi/2)
    Ts[3] = _dh_tf(q[2],    0, 0,    0)

    # Transform from arm to base.
    Ts[4] = T4
    Ts[5] = T5

    # Transform from end effector to arm.
    Ts[6]  = _dh_tf(q[3], 0,  d1,  np.pi/2)
    Ts[7]  = _dh_tf(q[4], a2, 0,   0)
    Ts[8]  = _dh_tf(q[5], a3, 0,   0)
    Ts[9]  = _dh_tf(q[6], 0,  d4,  np.pi/2)
    Ts[10] = _dh_tf(q[7], 0,  d5, -np.pi/2)
    Ts[11] = _dh_tf(q[8], 0,  d6,  0)

    return Ts


def forward(q):
    Ts = _calc_chain(q)
    w_T_e = np.eye(4)
    for T in Ts:
        w_T_e = w_T_e.dot(T)
    return w_T_e


def forward_chain(q):
    Ts = _calc_chain(q)
    w_T_ = [None] * 12

    w_T_[0] = Ts[0]
    for i in xrange(1, len(w_T_)):
        w_T_[i] = w_T_[i-1].dot(Ts[i])
    return w_T_


def _jacobians(q):
    # Base joints
    xb = q[0]
    yb = q[1]
    stb = np.sin(q[2])
    ctb = np.cos(q[2])

    # Arm joints
    sq1 = np.sin(q[3])
    cq1 = np.cos(q[3])
    sq2 = np.sin(q[4])
    cq2 = np.cos(q[4])
    sq3 = np.sin(q[5])
    cq3 = np.cos(q[5])
    sq4 = np.sin(q[6])
    cq4 = np.cos(q[6])
    sq5 = np.sin(q[7])
    cq5 = np.cos(q[7])
    sq6 = np.sin(q[8])
    cq6 = np.cos(q[8])

    # Base Jacobian
    Jb02 = a2*(-sq1*ctb - stb*cq1)*cq2 - a3*(-sq1*ctb - stb*cq1)*sq2*sq3 + a3*(-sq1*ctb - stb*cq1)*cq2*cq3 + d4*(-sq1*stb + cq1*ctb) + d5*(((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4) + d6*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5) - px*stb - py*ctb

    Jb12 = a2*(-sq1*stb + cq1*ctb)*cq2 - a3*(-sq1*stb + cq1*ctb)*sq2*sq3 + a3*(-sq1*stb + cq1*ctb)*cq2*cq3 + d4*(sq1*ctb + stb*cq1) + d5*(((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*cq3 - (sq1*stb - cq1*ctb)*sq3*cq2)*cq4) + d6*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5) + px*ctb - py*stb

    Jb = np.array([
        [1, 0, Jb02],
        [0, 1, Jb12],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 1]])

    # Arm
    Ja = np.zeros((6, 6))
    Ja[0,0] = a2*(-sq1*ctb - stb*cq1)*cq2 - a3*(-sq1*ctb - stb*cq1)*sq2*sq3 + a3*(-sq1*ctb - stb*cq1)*cq2*cq3 + d4*(-sq1*stb + cq1*ctb) + d5*(((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4) + d6*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5)
    Ja[0,1] = -a2*(-sq1*stb + cq1*ctb)*sq2 - a3*(-sq1*stb + cq1*ctb)*sq2*cq3 - a3*(-sq1*stb + cq1*ctb)*sq3*cq2 + d5*((-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4 + ((sq1*stb - cq1*ctb)*sq2*sq3 - (sq1*stb - cq1*ctb)*cq2*cq3)*cq4) + d6*(-(-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5
    Ja[0,2] = -a3*(-sq1*stb + cq1*ctb)*sq2*cq3 - a3*(-sq1*stb + cq1*ctb)*sq3*cq2 + d5*((-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*sq4 + ((sq1*stb - cq1*ctb)*sq2*sq3 - (sq1*stb - cq1*ctb)*cq2*cq3)*cq4) + d6*(-(-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5
    Ja[0,3] = d5*((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - ((-sq1*stb + cq1*ctb)*sq2*cq3 + (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4) + d6*((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*sq5
    Ja[0,4] = d6*((-(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*cq5 - (sq1*ctb + stb*cq1)*sq5)
    Ja[0,5] = 0

    Ja[1,0] = a2*(-sq1*stb + cq1*ctb)*cq2 - a3*(-sq1*stb + cq1*ctb)*sq2*sq3 + a3*(-sq1*stb + cq1*ctb)*cq2*cq3 + d4*(sq1*ctb + stb*cq1) + d5*(((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*cq3 - (sq1*stb - cq1*ctb)*sq3*cq2)*cq4) + d6*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5)
    Ja[1,1] = -a2*(sq1*ctb + stb*cq1)*sq2 - a3*(sq1*ctb + stb*cq1)*sq2*cq3 - a3*(sq1*ctb + stb*cq1)*sq3*cq2 + d5*(((-sq1*ctb - stb*cq1)*sq2*sq3 - (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*sq4) + d6*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*cq4)*sq5
    Ja[1,2] = -a3*(sq1*ctb + stb*cq1)*sq2*cq3 - a3*(sq1*ctb + stb*cq1)*sq3*cq2 + d5*(((-sq1*ctb - stb*cq1)*sq2*sq3 - (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4) + d6*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5
    Ja[1,3] = d5*((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4) + d6*((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5
    Ja[1,4] = d6*((-(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 - (sq1*stb - cq1*ctb)*sq5)
    Ja[1,5] = 0

    Ja[2,0] = 0
    Ja[2,1] = a2*cq2 - a3*sq2*sq3 + a3*cq2*cq3 + d5*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4) - d6*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5
    Ja[2,2] = -a3*sq2*sq3 + a3*cq2*cq3 + d5*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4) - d6*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5
    Ja[2,3] = d5*(-(sq2*sq3 - cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4) - d6*((-sq2*sq3 + cq2*cq3)*cq4 - (sq2*cq3 + sq3*cq2)*sq4)*sq5
    Ja[2,4] = -d6*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*cq5
    Ja[2,5] = 0

    Ja[3,0] = sq1*ctb + stb*cq1
    Ja[3,1] = sq1*ctb + stb*cq1
    Ja[3,2] = sq1*ctb + stb*cq1
    Ja[3,3] = (-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4
    Ja[3,4] = -((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5
    Ja[3,5] = -((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5

    Ja[4,0] = sq1*stb - cq1*ctb
    Ja[4,1] = sq1*stb - cq1*ctb
    Ja[4,2] = sq1*stb - cq1*ctb
    Ja[4,3] = (-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4
    Ja[4,4] = -((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (sq1*stb - cq1*ctb)*cq5
    Ja[4,5] = -((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (sq1*stb - cq1*ctb)*cq5

    Ja[5,0] = 0
    Ja[5,1] = 0
    Ja[5,2] = 0
    Ja[5,3] = -(-sq2*sq3 + cq2*cq3)*cq4 + (sq2*cq3 + sq3*cq2)*sq4
    Ja[5,4] = -((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*sq5
    Ja[5,5] = -((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*sq5

    return Ja, Jb


def jacobian(q):
    Ja, Jb = _jacobians(q)
    J = np.hstack([Jb, Ja])
    return J


def manipulability(q):
    Ja, _ = _jacobians(q)

    m2 = np.linalg.det(Ja.dot(Ja.T))

    # handle numerical errors pushing us slightly negative
    if m2 < 0:
        m2 = 0
    m = np.sqrt(m2)
    return m

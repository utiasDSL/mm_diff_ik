import sympy as sym
import numpy as np


def dh_tf(q, a, d, alpha):
    ''' Transformation matrix from D-H parameters. '''
    return sym.Matrix([
        [sym.cos(q), -sym.sin(q)*sym.cos(alpha),  sym.sin(q)*sym.sin(alpha), a*sym.cos(q)],
        [sym.sin(q),  sym.cos(q)*sym.cos(alpha), -sym.cos(q)*sym.sin(alpha), a*sym.sin(q)],
        [0,           sym.sin(alpha),             sym.cos(alpha),            d],
        [0,           0,                          0,                         1]])


def dh_tf_np(q, a, d, alpha):
    ''' Numerical transformation matrix from D-H parameters. '''
    return np.array([
        [np.cos(q), -np.sin(q)*np.cos(alpha),  np.sin(q)*np.sin(alpha), a*np.cos(q)],
        [np.sin(q),  np.cos(q)*np.cos(alpha), -np.cos(q)*np.sin(alpha), a*np.sin(q)],
        [0,          np.sin(alpha),            np.cos(alpha),           d],
        [0,          0,                        0,                       1]])


def R_t_from_T(T):
    ''' Extract rotation and translation from transform. '''
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    return R, t


def _as_np(M):
    ''' Convert sympy matrix to numpy array. '''
    return np.array(M).astype(np.float64)

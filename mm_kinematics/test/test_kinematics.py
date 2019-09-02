import numpy as np

import mm_kinematics.kinematics as kinematics
from mm_kinematics.symbolic import SymbolicKinematics


np.random.seed(0)


sym_kin = SymbolicKinematics()


def test_comp_fk():
    ''' Ensure forward kinematics match between symbolic and numerical
        implementations. '''
    q = np.zeros(9)
    P_sym = sym_kin.forward(q)
    P_np = kinematics.forward(q)
    assert np.allclose(P_sym, P_np)

    q = np.random.random(9)
    P_sym = sym_kin.forward(q)
    P_np = kinematics.forward(q)
    assert np.allclose(P_sym, P_np)


def test_comp_jacobian():
    ''' Ensure Jacobians match between symbolic and numerical
        implementations. '''
    q = np.zeros(9)
    J_sym = sym_kin.jacobian(q)
    J_np = kinematics.jacobian(q)
    assert np.allclose(J_sym, J_np)

    q = np.random.random(9)
    J_sym = sym_kin.jacobian(q)
    J_np = kinematics.jacobian(q)
    assert np.allclose(J_sym, J_np)

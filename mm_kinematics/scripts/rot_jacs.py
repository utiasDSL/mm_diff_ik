#!/usr/bin/env python2
from mm_kinematics.symbolic import SymbolicKinematics


def write_sym_jac():
    kin = SymbolicKinematics()
    kin.write_sym_orientation_jacobian('rot_jac.txt', fmt='c++')


if __name__ == '__main__':
    write_sym_jac()

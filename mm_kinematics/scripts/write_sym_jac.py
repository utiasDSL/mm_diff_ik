#!/usr/bin/env python2
from mm_kinematics import ThingKinematics


def write_sym_jac():
    kin = ThingKinematics()
    kin.write_sym_jac('jac.txt', fmt='py')


if __name__ == '__main__':
    write_sym_jac()

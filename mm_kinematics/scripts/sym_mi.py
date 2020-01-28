#!/usr/bin/env python2
from mm_kinematics.symbolic import SymbolicKinematics
import IPython


def main():
    kin = SymbolicKinematics()
    mi = kin.calc_sym_mi()
    IPython.embed()


if __name__ == '__main__':
    main()

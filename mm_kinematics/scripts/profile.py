#!/usr/bin/env python2
from __future__ import print_function

import timeit
import numpy as np

from mm_kinematics import ThingKinematics


kin = ThingKinematics()


def test_sym():
    # TODO could handle more range later
    q = np.random.random(9)
    P = kin.calc_fk(q)
    return P


def test_np():
    q = np.random.random(9)
    P = kin.calc_fk_np(q)
    return P


def main():
    N_sym = 10
    N_np = 1000

    t_np = timeit.timeit('test_np()', setup='from __main__ import test_np', number=N_np)
    t_sym = timeit.timeit('test_sym()', setup='from __main__ import test_sym', number=N_sym)

    r_np = t_np / N_np
    r_sym = t_sym / N_sym

    r = r_sym / r_np

    print('Symbolic  = {}s/call'.format(r_sym))
    print('Numerical = {}s/call'.format(r_np))
    print('Numerical is {}x faster than symbolic'.format(r))


if __name__ == '__main__':
    main()

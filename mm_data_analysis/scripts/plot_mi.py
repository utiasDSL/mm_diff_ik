#!/usr/bin/env python2
# Analyse and plot manipulability maximization objective experimental results
from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt
import mm_data_analysis.manipulability as manip


BAG0 = 'bags/2019-09-10/baseline/line_v0.1_2019-09-10-14-03-05.bag'

BAGS1 = [
    'bags/2019-09-12/mi/line_mi1_w1_2019-09-12-13-47-08.bag',
    'bags/2019-09-12/mi/line_mi1_w10_2019-09-12-13-49-22.bag',
    'bags/2019-09-12/mi/line_mi1_w100_2019-09-12-13-52-07.bag',
    'bags/2019-09-12/mi/line_mi1_w1000_2019-09-12-13-54-19.bag',
    'bags/2019-09-12/mi/line_mi1_w10000_2019-09-12-13-57-52.bag'
]

BAGS2 = [
    'bags/2019-09-12/mi/line_mi2_w1_2019-09-12-14-02-11.bag',
    'bags/2019-09-12/mi/line_mi2_w10_2019-09-12-14-04-07.bag',
    'bags/2019-09-12/mi/line_mi2_w100_2019-09-12-14-05-54.bag',
    'bags/2019-09-12/mi/line_mi2_w1000_2019-09-12-14-08-15.bag',
    'bags/2019-09-12/mi/line_mi2_w10000_2019-09-12-14-11-17.bag'
]

WEIGHTS = [1, 10, 100, 1000, 10000]


def main():
    t0, mi0, _ = manip.parse_bag(0, BAG0)

    t1 = []
    mi1 = []
    f1 = []

    t2 = []
    mi2 = []
    f2 = []

    for i in xrange(len(WEIGHTS)):
        t, mi, f = manip.parse_bag(WEIGHTS[i], BAGS1[i])
        t1.append(t)
        mi1.append(mi)
        f1.append(f)

        t, mi, f = manip.parse_bag(WEIGHTS[i], BAGS2[i])
        t2.append(t)
        mi2.append(mi)
        f2.append(f)

    # figure size in inches - fixed to 3.25" width
    fig = plt.figure(figsize=(3.25, 2))
    plt.rcParams.update({'font.size': 8,
                         'text.usetex': True})

    cs = ['r', 'b', 'g', 'c', 'm', 'k']
    plt.plot(t0, mi0, label='$\\alpha=0$', linewidth=2, c=cs[0])
    for i in xrange(len(t1)):
        plt.plot(t1[i], mi1[i], label='$\\alpha={}$'.format(WEIGHTS[i]), linewidth=2, c=cs[i+1])

    plt.legend(labelspacing=0, borderpad=0.1, ncol=2, loc=0)
    plt.xlabel('Time (s)')
    plt.ylabel('Manipulability Index')

    print('avg freq (1st) = {}'.format(np.mean(f1)))
    print('avg freq (2nd) = {}'.format(np.mean(f2)))

    fig.tight_layout(pad=0.1)
    fig.savefig('mi.pdf')
    plt.show()


if __name__ == '__main__':
    main()

#!/usr/bin/env python2
# Analyse and plot manipulability maximization objective experimental results
from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt
import mm_data_analysis.manipulability as manip


def main():
    t0, mi0, _ = manip.parse_bag(BAG0)

    t1 = []
    mi1 = []
    f1 = []

    t2 = []
    mi2 = []
    f2 = []

    for i in xrange(len(WEIGHTS)):
        t, mi, f = manip.parse_bag(BAGS1[i])
        t1.append(t)
        mi1.append(mi)
        f1.append(f)

        t, mi, f = manip.parse_bag(BAGS2[i])
        t2.append(t)
        mi2.append(mi)
        f2.append(f)

    # figure size in inches - fixed to 3.25" width
    fig = plt.figure(figsize=(3.25, 1.5))
    plt.rcParams.update({'font.size': 8,
                         'text.usetex': True,
                         'legend.fontsize': 8})

    cs = ['r', 'b', 'g', 'c', 'm', 'k']
    plt.plot(t0, mi0, label='$w_m=0$', linewidth=2, c=cs[0])
    for i in xrange(len(t1)):
        plt.plot(t1[i], mi1[i], label='$w_m={}$'.format(WEIGHTS[i]), linewidth=2, c=cs[i+1])

    plt.legend(labelspacing=0, borderpad=0.1, ncol=2, loc=0, facecolor='white', framealpha=1)
    plt.xlabel('Time (s)')
    plt.ylabel('Manipulability Index')

    # plt.yticks([0.02, 0.03, 0.04, 0.05])

    print('avg freq (1st) = {}'.format(np.mean(f1)))
    print('avg freq (2nd) = {}'.format(np.mean(f2)))

    fig.tight_layout(pad=0.1)
    fig.savefig('mi.pdf')
    plt.show()


if __name__ == '__main__':
    main()

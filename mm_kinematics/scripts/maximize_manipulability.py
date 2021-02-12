#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize

from mm_kinematics import KinematicModel


N = 10000


def main():
    np.random.seed(1)
    q0 = np.random.random(9) * 2 * np.pi - np.pi

    model = KinematicModel()

    def objective(q):
        return -model.manipulability(q)

    # 1. try using generic optimization from scipy
    res = optimize.minimize(objective, q0)
    m_opt = model.manipulability(res.x)

    # 2. try using random sampling
    ms = np.zeros(N)
    for i in xrange(N):
        q = np.random.random(9) * 2 * np.pi - np.pi
        ms[i] = model.manipulability(q)

    print("Optimized MI = {}".format(m_opt))
    print("Max MI after {} trials = {}".format(N, np.max(ms)))

    plt.hist(ms, 100)
    plt.xlabel("Manipulability")
    plt.ylabel("Count")
    plt.show()


if __name__ == "__main__":
    main()

#!/usr/bin/env python2
import numpy as np
from scipy import optimize
import matplotlib.pyplot as plt
import mm_kinematics.kinematics as kinematics
import IPython

# Find maximum manipulability index of the Thing.

q0 = np.random.random(9) * 2 * np.pi - np.pi
N = 10000


def objective(q):
    return -kinematics.manipulability(q)


res = optimize.minimize(objective, q0)
m_opt = kinematics.manipulability(res.x)

ms = np.zeros(N)

for i in xrange(N):
    q = np.random.random(9) * 2 * np.pi - np.pi
    ms[i] = kinematics.manipulability(q)

print('Optimized MI = {}'.format(m_opt))
print('Max MI after {} trials = {}'.format(N, np.max(ms)))

plt.hist(ms, 100)
plt.xlabel('Manipulability')
plt.ylabel('Count')
plt.show()

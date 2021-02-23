from collections import deque
import numpy as np


class AveragingFilter(object):
    ''' Low-pass unweighted averaging filter. '''
    def __init__(self, max_size):
        self.max_size = max_size
        self.data = deque()

    def filter(self, value):
        self.data.append(value)
        if len(self.data) > self.max_size:
            self.data.popleft()

        total = 0
        for datum in self.data:
            total += datum
        return total / len(self.data)


class ExponentialSmoother(object):
    ''' 1st order exponentional smoother. '''
    def __init__(self, tau, x0):
        self.tau = tau
        self.prev = x0

    def next(self, measured, dt):
        c = 1.0 - np.exp(-dt / self.tau)
        state = c * measured + (1 - c) * self.prev
        self.prev = state
        return state

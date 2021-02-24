import numpy as np


def bound_array(a, lb, ub):
    """Elementwise bound array above and below."""
    return np.minimum(np.maximum(a, lb), ub)


def rotation2d(angle):
    """Principal rotation matrix about the z axis."""
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, -s], [s, c]])

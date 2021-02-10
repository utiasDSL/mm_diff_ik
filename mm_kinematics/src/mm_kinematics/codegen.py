import re
import numpy as np


def replace_sin_cos(s):
    def sin_repl(m):
        return "s" + m.group(1)

    def cos_repl(m):
        return "c" + m.group(1)

    s = re.sub("sin\(([a-z0-9]+)\)", sin_repl, s)
    s = re.sub("cos\(([a-z0-9]+)\)", cos_repl, s)
    return s


def encode_matrix(J):
    """Encode matrix as a string."""
    J_str = np.empty(J.shape, dtype=object)
    for i in range(J.shape[0]):
        for j in range(J.shape[1]):
            s = str(J[i, j])
            J_str[i, j] = replace_sin_cos(s)
    return J_str

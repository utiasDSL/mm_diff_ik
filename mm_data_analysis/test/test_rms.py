import pytest
import numpy as np
from mm_data_analysis.util import rms


def test_rms_ones():
    x1 = np.array([1, 1, 1])
    y1 = rms(x1)
    assert y1 == 1

    # negatives don't matter
    x2 = np.array([-1, 1, -1])
    y2 = rms(x2)
    assert y1 == y2

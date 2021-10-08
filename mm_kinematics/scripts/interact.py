#!/usr/bin/env python2
import numpy as np
from mm_kinematics import SymbolicKinematicModel

import IPython


def main():
    np.set_printoptions(precision=3, suppress=True)
    q = np.array(
        [
            0,
            0,
            0,
            0.0,
            -0.75 * np.pi,
            -0.5 * np.pi,
            -0.25 * np.pi,
            -0.5 * np.pi,
            0.5 * np.pi,
        ]
    )
    model = SymbolicKinematicModel()
    J = model.jacobian(q)

    IPython.embed()


if __name__ == "__main__":
    main()

#!/usr/bin/env python2
"""Save lambdified symbolic kinematic functions for fast loading later."""
import os
import sys
import dill
from mm_kinematics import SymbolicKinematicModel, codegen

import IPython


def main():
    model = SymbolicKinematicModel()

    IPython.embed()


if __name__ == '__main__':
    main()

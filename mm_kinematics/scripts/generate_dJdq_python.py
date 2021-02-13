#!/usr/bin/env python2
import os
import sys
import dill
import sympy

from mm_kinematics import SymbolicKinematicModel, codegen, JOINT_NAMES


DEFAULT_FILE_NAME = "dJdq.pkl"


def main():
    if len(sys.argv) < 2:
        path = os.path.dirname(__file__)
        path = codegen.get_default_file_path(path, DEFAULT_FILE_NAME)
    else:
        path = sys.argv[1]

    model = SymbolicKinematicModel()

    # generate the symbolic derivatives
    Ja = model.parameterize(model.J[:, 3:])
    dJa_dqs = [Ja.diff(qi) for qi in model.q]

    dJa_dq_funcs = {}
    for qi, dJa_dqi in zip(JOINT_NAMES[2:-1], dJa_dqs[2:-1]):
        func_name = "dJa_d" + qi
        dJa_dq_funcs[func_name] = sympy.lambdify([model.q], dJa_dqi)

    dill.settings["recurse"] = True
    with open(path, "wb") as f:
        dill.dump(dJa_dq_funcs, f)

    print("Wrote Python dJdq functions to {}.".format(path))


if __name__ == "__main__":
    main()

#!/usr/bin/env python2
"""Save lambdified symbolic kinematic functions for fast loading later."""
import os
import sys
import dill
from mm_kinematics import SymbolicKinematicModel, codegen


DEFAULT_FILE_NAME = "kinematics.pkl"


def main():
    if len(sys.argv) < 2:
        path = os.path.dirname(__file__)
        path = codegen.get_default_file_path(path, DEFAULT_FILE_NAME)
    else:
        path = sys.argv[1]

    model = SymbolicKinematicModel()
    kinematic_funcs = {
        "jacobian": model.jacobian,
        "calc_T_w_tool": model.calc_T_w_tool,
        "calc_T_w_ee": model.calc_T_w_ee,
        "calc_T_w_palm": model.calc_T_w_palm,
        "calc_T_w_ft": model.calc_T_w_ft,
        "calc_T_w_base": model.calc_T_w_base,
        "calc_chain": model.calc_chain,
    }

    dill.settings["recurse"] = True
    with open(path, "wb") as f:
        dill.dump(kinematic_funcs, f)

    print("Wrote Python kinematic functions to {}.".format(path))


if __name__ == '__main__':
    main()

#!/usr/bin/env python2
import sys
import numpy as np
import re

from mm_kinematics.symbolic import KinematicModel
import mm_kinematics.codegen as codegen


TEXT_BEFORE = """
#include <ros/ros.h>
#include <Eigen/Eigen>

#include "mm_kinematics/kinematics.h"

namespace mm {

void Kinematics::jacobians(const JointVector& q, ArmJacobianMatrix& Ja, BaseJacobianMatrix& Jb) {
    // Base joints
    double xb = q(0);
    double yb = q(1);
    double stb = std::sin(q(2));
    double ctb = std::cos(q(2));

    // Arm joints
    double sq1 = std::sin(q(3));
    double cq1 = std::cos(q(3));
    double sq2 = std::sin(q(4));
    double cq2 = std::cos(q(4));
    double sq3 = std::sin(q(5));
    double cq3 = std::cos(q(5));
    double sq4 = std::sin(q(6));
    double cq4 = std::cos(q(6));
    double sq5 = std::sin(q(7));
    double cq5 = std::cos(q(7));
    double sq6 = std::sin(q(8));
    double cq6 = std::cos(q(8));

"""

TEXT_AFTER = """
}

} // namespace mm
"""


def main():
    if len(sys.argv) < 2:
        print('Usage: generate_jacobian_cpp.py path')
        return

    path = sys.argv[1]
    model = KinematicModel()
    J_str = codegen.encode_matrix(model.J)
    msg = "{}({},{}) = {};\n"

    with open(path, 'w+') as f:
        f.write(TEXT_BEFORE)

        f.write("// Base Jacobian\n")
        for i in range(6):
            for j in range(3):
                f.write(msg.format('Jb', i, j, J_str[i, j]))

        f.write("\n// Arm Jacobian\n")
        for i in range(6):
            for j in range(6):
                f.write(msg.format('Ja', i, j, J_str[i, j+3]))

        f.write(TEXT_AFTER)


if __name__ == '__main__':
    main()

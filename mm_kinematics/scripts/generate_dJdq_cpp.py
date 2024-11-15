#!/usr/bin/env python2
import os
import sys
from mm_kinematics import SymbolicKinematicModel, codegen, JOINT_NAMES


DEFAULT_FILE_NAME = "dJdq.cpp"


TEXT_BEFORE = """
// Autogenerated by generate_dJdq_cpp.py -- do not edit by hand.
#include <ros/ros.h>
#include <Eigen/Eigen>

#include "mm_kinematics/kinematics.h"

namespace mm {
""".lstrip()


TEXT_AFTER = """
} // namespace mm
"""

FUNCTION_VARS = """
    // Base variables
    double stb = std::sin(q(2));
    double ctb = std::cos(q(2));

    // Arm variables
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

TEXT_BEFORE_SINGLE_FUNC = """
// Autogenerated by generate_dJdq_cpp.py -- do not edit by hand.
#include <ros/ros.h>
#include <Eigen/Eigen>

#include "mm_kinematics/kinematics.h"

namespace mm {

void Kinematics::calc_dJa_dq(const JointVector& q,
                             ArmJacobianMatrix& dJa_dtb,
                             ArmJacobianMatrix& dJa_dq1,
                             ArmJacobianMatrix& dJa_dq2,
                             ArmJacobianMatrix& dJa_dq3,
                             ArmJacobianMatrix& dJa_dq4,
                             ArmJacobianMatrix& dJa_dq5) {
    // Base variables
    double stb = std::sin(q(2));
    double ctb = std::cos(q(2));

    // Arm variables
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
""".lstrip()


def stringify_matrix(M, name):
    M_str = codegen.encode_matrix(M)
    msg = "  {}({},{}) = {};"

    items = []
    for i in range(6):
        for j in range(6):
            items.append(msg.format(name, i, j, M_str[i, j]))
    return '\n'.join(items)


def main():
    if len(sys.argv) < 2:
        path = os.path.dirname(__file__)
        path = codegen.get_default_file_path(path, DEFAULT_FILE_NAME)
    else:
        path = sys.argv[1]

    model = SymbolicKinematicModel()

    # generate the symbolic derivatives
    Ja = model.J[:, 3:]
    dJa_dqs = [Ja.diff(qi) for qi in model.q]

    items = [TEXT_BEFORE_SINGLE_FUNC]
    for qi, dJa_dqi in zip(JOINT_NAMES[2:-1], dJa_dqs[2:-1]):
        code = stringify_matrix(dJa_dqi, 'dJa_d' + qi) + '\n'
        items.append(code)
    items.append('}\n\n} // namespace mm')

    with open(path, 'w+') as f:
        f.write('\n'.join(items))

    print("Wrote C++ dJdq function to {}.".format(path))


if __name__ == '__main__':
    main()

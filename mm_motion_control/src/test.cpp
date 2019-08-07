#include <Eigen/Eigen>
#include <iostream>

#include "mm/mm.h"
#include "mm/kinematics.h"
#include "mm/interp.h"
#include "mm/optimize.h"

using namespace Eigen;

// TODO
// 1. move these tests to actual gtest tests
// 2. make them better


void test_qp() {
    mm::JointVector q = mm::JointVector::Zero();
    mm::Vector6d ee_vel;
    ee_vel << 1, 0, 0, 0, 0, 0;

    mm::JointVector dq_opt;

    mm::IKOptimizer optimizer;
    bool ret = optimizer.solve(q, ee_vel, dq_opt);

    ROS_INFO_STREAM("dq_opt = " << dq_opt);
    ROS_INFO_STREAM("ret = " << ret);
}


int main() {
    test_qp();
    return 0;
}

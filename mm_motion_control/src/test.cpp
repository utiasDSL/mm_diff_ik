#include <Eigen/Eigen>
#include <iostream>

#include "mm/mm.h"
#include "mm/kinematics.h"
#include "mm/interp.h"
#include "mm/optimize.h"

using namespace Eigen;

typedef Matrix<double, 1, 1> Vector1d;

// TODO
// 1. move these tests to actual gtest tests
// 2. make them better

void test_interp() {
    Vector1d x1; x1 << 4;
    Vector1d x2; x2 << 10;
    Vector1d dx1; dx1 << 1;
    Vector1d dx2; dx2 << 13;

    mm::CubicInterp<1> interp;
    interp.interpolate(1, 2, x1, x2, dx1, dx2);

    Vector1d x, dx;

    interp.sample(1.5, x, dx);

    std::cout << x << " " << dx << std::endl;
}

void test_qp() {
    mm::QVector q = mm::QVector::Zero();
    mm::Vector6d ee_vel;
    ee_vel << 1, 0, 0, 0, 0, 0;

    mm::QVector dq_opt;

    mm::IKOptimizer optimizer;
    bool ret = optimizer.solve(q, ee_vel, dq_opt);

    ROS_INFO_STREAM("dq_opt = " << dq_opt);
    ROS_INFO_STREAM("ret = " << ret);
}


int main() {
    // test_qp();
    // test_interp();
    return 0;
}

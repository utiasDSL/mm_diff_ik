#include <iostream>
#include <chrono>

#include <gtest/gtest.h>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/pose_control/optimizer.h"



using namespace mm;
using namespace Eigen;


TEST(OptimizeTestSuite, testLinearizedManipulability) {
    // goal is verify that the manipulability objective is working as expected
    JointVector q = JointVector::Zero();
    q << -1.0, -1.0, 0.0, 0.0, -2.3562, -1.5708, -2.3562, -1.5708, 1.5708;
    // q(4) = -M_PI*0.75;
    // q(5) = -M_PI_2;

    double m = Kinematics::manipulability(q);

    IKOptimizer optimizer;
    JointVector dm, dm2;
    JointMatrix Hm;

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; ++i) {
        q = JointVector::Random();
        optimizer.linearize_manipulability1(q, dm, STEP_SIZE);
    }

    auto t1 = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 1000; ++i) {
        q = JointVector::Random();
        Kinematics::manipulability_gradient(q, dm2);
    }
    auto t2 = std::chrono::high_resolution_clock::now();

    using milli = std::chrono::milliseconds;
    std::cout << "numeric took "
              << std::chrono::duration_cast<milli>(t1 - t0).count()
              << " ms." << std::endl;
    std::cout << "analytic took "
              << std::chrono::duration_cast<milli>(t2 - t1).count()
              << " ms." << std::endl;

    // std::cout << m << std::endl;
    // std::cout << "dm = " << dm << std::endl;
    // std::cout << "dm2 = " << dm2 << std::endl;
    // std::cout << Hm << std::endl;
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

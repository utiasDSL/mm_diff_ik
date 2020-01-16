#include <iostream>

#include <gtest/gtest.h>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/pose_control/optimizer.h"



using namespace mm;
using namespace Eigen;


TEST(OptimizeTestSuite, testLinearizedManipulability) {
    // goal is verify that the manipulability objective is working as expected
    JointVector q = JointVector::Zero();
    q(4) = -M_PI*0.75;
    q(5) = -M_PI_2;

    double m = Kinematics::manipulability(q);

    IKOptimizer optimizer;
    JointVector dm;
    JointMatrix Hm;
    optimizer.linearize_manipulability2(q, dm, Hm, STEP_SIZE);

    // std::cout << m << std::endl;
    // std::cout << dm << std::endl;
    std::cout << Hm << std::endl;
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

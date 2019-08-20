#include <ros/ros.h>
#include <gtest/gtest.h>

#include "mm_kinematics/kinematics.h"

#include <iostream>


using namespace mm;
using namespace Eigen;


void test_jac_finite_diff(const JointVector& q, const double eps) {
    JacobianMatrix J;
    Kinematics::jacobian(q, J);

    for (int i = 0; i < 9; ++i) {
        JointVector epsv = JointVector::Zero();
        epsv(i) = eps;

        Affine3d T1, T2;
        mm::Kinematics::forward(q + epsv, T1);
        mm::Kinematics::forward(q - epsv, T2);

        // Calculate numerical approximation of derivative via central finite
        // difference and compare to analytical derivative from the Jacobian.
        // TODO we are only comparing the linear parts right now
        Vector3d approx = (T1.translation() - T2.translation()) / (2*eps);
        Vector3d real = J.block<3,1>(0,i);

        EXPECT_TRUE((approx - real).norm() < eps);
    }

}


TEST(KinematicsTestSuite, testJac1) {
    // All joints are zero.
    JointVector q = JointVector::Zero();
    double eps = 1e-8;
    test_jac_finite_diff(q, eps);
}


TEST(KinematicsTestSuite, testJac2) {
    // Arm is in "ready position"
    JointVector q = JointVector::Zero();
    q(4) = -M_PI * 0.75;
    q(5) = -M_PI_2;
    double eps = 1e-8;
    test_jac_finite_diff(q, eps);
}


TEST(KinematicsTestSuite, testJac3) {
    // Base is moved.
    JointVector q = JointVector::Zero();
    q(0) = 1.0;
    q(1) = -0.5;
    q(2) = M_PI_2;
    double eps = 1e-8;
    test_jac_finite_diff(q, eps);
}


TEST(KinematicsTestSuite, testManipulability) {
    double eps = 1e-10;
    JointVector q = JointVector::Zero();

    // All zeros is singular.
    double m1 = Kinematics::manipulability(q);
    EXPECT_TRUE(0 <= m1 && m1 < eps);

    q(4) = -0.75 * M_PI;
    q(5) = -M_PI_2;

    // Moving the above joints is still singular.
    double m2 = Kinematics::manipulability(q);
    EXPECT_TRUE(0 <= m2 && m2 < eps);

    q(7) = M_PI_4;

    // Finally non-singular.
    double m3 = Kinematics::manipulability(q);
    EXPECT_TRUE(0.0134 < m3 && m3 < 0.0135);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

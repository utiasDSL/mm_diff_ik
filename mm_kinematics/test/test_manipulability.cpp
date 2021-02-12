#include <iostream>
#include <chrono>

#include <gtest/gtest.h>

#include <mm_kinematics/kinematics.h>



using namespace mm;
using namespace Eigen;


TEST(ManipulabilityTestSuite, testManipulability) {
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
    EXPECT_TRUE(m3 > eps);
}


TEST(ManipulabilityTestSuite, testManipulabilityGradientAccuracy) {
    JointVector q;
        q << -1.0, -1.0, 0.0, 0.0, -2.3562, -1.5708, -2.3562, -1.5708, 1.5708;
    // JointVector q = JointVector::Zero();

    double h = 1e-4;
    JointVector dm_analytic, dm_numeric;
    Kinematics::manipulability_gradient_analytic(q, dm_analytic);
    Kinematics::manipulability_gradient_numeric(q, dm_numeric, h);

    // std::cout << "dm_n = " << dm_numeric << std::endl;
    // std::cout << "dm_a = " << dm_analytic << std::endl;

    // Accuracy of numeric solution depends on size of h.
    // Increasing h to 1e-3 causes this test to fail.
    EXPECT_NEAR((dm_analytic - dm_numeric).norm(), 0, 1e-8);
}


TEST(ManipulabilityTestSuite, testManipulabilityGradientSpeed) {
    // goal is verify that the manipulability objective is working as expected
    JointVector q = JointVector::Zero();
    q << -1.0, -1.0, 0.0, 0.0, -2.3562, -1.5708, -2.3562, -1.5708, 1.5708;
    // q(4) = -M_PI*0.75;
    // q(5) = -M_PI_2;

    double h = 1e-3;
    double m = Kinematics::manipulability(q);

    JointVector dm, dm2;

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; ++i) {
        q = JointVector::Random();
        Kinematics::manipulability_gradient_numeric(q, dm, h);
    }

    auto t1 = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 1000; ++i) {
        q = JointVector::Random();
        Kinematics::manipulability_gradient_analytic(q, dm2);
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

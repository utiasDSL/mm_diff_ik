#include "mm/mm.h"
#include "mm/kinematics.h"

#include <gtest/gtest.h>


using namespace mm;
using namespace Eigen;


void test_jac_finite_diff(const QVector& q, const double eps) {
    JacMatrix J;
    Kinematics::jacobian(q, J);

    for (int i = 0; i < 9; ++i) {
        QVector epsv = QVector::Zero();
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
    QVector q = QVector::Zero();
    double eps = 1e-8;
    test_jac_finite_diff(q, eps);
}


TEST(KinematicsTestSuite, testJac2) {
    // Arm is in "ready position"
    QVector q = QVector::Zero();
    q(4) = -M_PI * 0.75;
    q(5) = -M_PI_2;
    double eps = 1e-8;
    test_jac_finite_diff(q, eps);
}


TEST(KinematicsTestSuite, testJac3) {
    // Base is moved.
    QVector q = QVector::Zero();
    q(0) = 1.0;
    q(1) = -0.5;
    q(2) = M_PI_2;
    double eps = 1e-8;
    test_jac_finite_diff(q, eps);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

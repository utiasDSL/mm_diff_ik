#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>

#include "mm_math_util/interp.h"
#include "mm_math_util/rotation.h"


using namespace mm;
using namespace Eigen;

const double EPS = 1e-10;


typedef Matrix<double, 1, 1> Vector1d;


TEST(InterpolationTestSuite, testInterpNoChange) {
    double t1 = 0;
    double t2 = 1;

    Vector1d x1 = Vector1d::Ones();
    Vector1d x2 = Vector1d::Ones();

    Vector1d dx1 = Vector1d::Zero();
    Vector1d dx2 = Vector1d::Zero();

    CubicInterp<1> interp;
    interp.interpolate(t1, t2, x1, x2, dx1, dx2);

    double t = 0.5;
    Vector1d xs, dxs;
    interp.sample(t, xs, dxs);

    // No change in position or velocity.
    EXPECT_TRUE((xs-x1).norm() < EPS);
    EXPECT_TRUE((dxs-dx1).norm() < EPS);

    // .sample(.) should return true when within the interpolation range and
    // false outside it.
    EXPECT_TRUE(interp.sample(0.5, xs, dxs));
    EXPECT_FALSE(interp.sample(1.5, xs, dxs));
}


// Move to new position and stop.
TEST(InterpolationTestSuite, testInterpMoveAndStop) {
    double t1 = 0;
    double t2 = 1;

    Vector1d x1 = Vector1d::Zero();
    Vector1d x2 = Vector1d::Ones();

    Vector1d dx1 = Vector1d::Zero();
    Vector1d dx2 = Vector1d::Zero();

    CubicInterp<1> interp;
    interp.interpolate(t1, t2, x1, x2, dx1, dx2);

    double t = 0.5;
    Vector1d x_sample, dx_sample;
    interp.sample(t, x_sample, dx_sample);

    // Midpoint
    EXPECT_TRUE(fabs(x_sample(0)-0.5)  < EPS);
    EXPECT_TRUE(fabs(dx_sample(0)-1.5) < EPS);
}


TEST(InterpolationTestSuite, testInterpMultiDimensional) {
    double t1 = 0;
    double t2 = 1;

    Vector3d x1 = Vector3d::Zero();
    Vector3d x2; x2 << -1, 0, 1;

    Vector3d dx1 = Vector3d::Zero();
    Vector3d dx2 = Vector3d::Zero();

    CubicInterp<3> interp;
    interp.interpolate(t1, t2, x1, x2, dx1, dx2);

    double t = 0.5;
    Vector3d x_sample, dx_sample;
    interp.sample(t, x_sample, dx_sample);

    Vector3d x_expected; x_expected << -0.5, 0, 0.5;
    Vector3d dx_expected; dx_expected << -1.5, 0, 1.5;

    // Midpoint
    EXPECT_TRUE((x_sample - x_expected).norm() < EPS);
    EXPECT_TRUE((dx_sample - dx_expected).norm() < EPS);
}


TEST(InterpolationTestSuite, testInterpBigTime) {
    double t1 = 1e6;
    double t2 = t1 + 0.1;

    Vector3d v; v << 0.05, 0, 0;

    Vector3d x1 = Vector3d::Zero();
    Vector3d x2 = x1 + 0.1*v;

    CubicInterp<3> interp;
    interp.interpolate(t1, t2, x1, x2, v, v);

    double t = t1 + 0.05;
    Vector3d x_sample, v_sample;
    interp.sample(t, x_sample, v_sample);

    Vector3d x_expected; x_expected << 0.0025, 0, 0;
    Vector3d v_expected; v_expected << 0.05,   0, 0;

    // Midpoint
    EXPECT_TRUE((x_sample - x_expected).norm() < EPS);
    EXPECT_TRUE((v_sample - v_expected).norm() < EPS);
}


TEST(InterpolationTestSuite, testSlerp2D) {
    Eigen::Vector2d a, b, c, c_ex;
    a << 1, 0;

    // vectors with same direction
    c = slerp2d(a, a, 0);
    EXPECT_TRUE((c - a).norm() < EPS);

    c = slerp2d(a, a, 0.5);
    EXPECT_TRUE((c - a).norm() < EPS);

    c = slerp2d(a, a, 1);
    EXPECT_TRUE((c - a).norm() < EPS);

    // vectors with opposite direction
    b << -1, 0;
    c = slerp2d(a, b, 0);
    EXPECT_TRUE((c - a).norm() < EPS);

    c = slerp2d(a, b, 1);
    EXPECT_TRUE((c - b).norm() < EPS);

    c = slerp2d(a, b, 0.5);
    c_ex << 0, 1;
    EXPECT_TRUE((c - c_ex).norm() < EPS);

    c = slerp2d(a, b, -0.5);
    c_ex << 0, -1;
    EXPECT_TRUE((c - c_ex).norm() < EPS);

    // positive dot product
    b << 1, 1;
    b.normalize();
    c = slerp2d(a, b, 0.5);
    c_ex = rotation2d(0.5*M_PI_4) * a;
    EXPECT_NEAR((c - c_ex).norm(), 0, EPS);

    // should be symmetric
    c = slerp2d(b, a, 0.5);
    EXPECT_NEAR((c - c_ex).norm(), 0, EPS);

    // t can also be > 1
    c = slerp2d(a, b, 2);
    c_ex << 0, 1;
    EXPECT_NEAR((c - c_ex).norm(), 0, EPS);

    // negative dot product
    b << -1, 1;
    c = slerp2d(a, b, 2.0/3.0);
    c_ex << 0, 1;
    EXPECT_NEAR((c - c_ex).norm(), 0, EPS);

    // check it works in other quadrants
    a << 0, -1;
    b << -1, 0;
    c = slerp2d(a, b, 0.25);
    c_ex << rotation2d(-0.5*M_PI_4) * a;
    EXPECT_NEAR((c - c_ex).norm(), 0, EPS);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

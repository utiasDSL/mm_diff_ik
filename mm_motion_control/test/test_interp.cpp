#include "mm/mm.h"
#include "mm/interp.h"

#include <iostream>

#include <gtest/gtest.h>


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


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

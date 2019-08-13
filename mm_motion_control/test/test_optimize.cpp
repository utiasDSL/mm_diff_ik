#include "mm/optimize.h"

#include <gtest/gtest.h>
#include <iostream>


using namespace mm;
using namespace Eigen;


Matrix3d A;
Vector3d b;
double c;


double f(const Vector3d& x) {
    // for some reason g++ has a problem with these all being in a
    // single expression
    double p = 0.5 * x.transpose() * A * x;
    double q = b.transpose() * x;
    return p + q + c;
}

Vector3d grad(const Vector3d& x) {
    return 0.5*(A+A.transpose())*x + b;
}

Matrix3d hess(const Vector3d& x) {
    return 0.5*(A+A.transpose());
}

// Check that all entries of matrix m have an absolute value smaller than eps.
template<int r, int c>
bool check_entries(Matrix<double, r, c>& m, double eps) {
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            if (fabs(m(i,j)) > eps) {
                return false;
            }
        }
    }
    return true;
}

TEST(OptimizeTestSuite, testApproxGradient) {
    double h = 1e-7;
    Vector3d x = Vector3d::Ones();

    Vector3d g_approx;
    mm::approx_gradient<3>(f, h, x, g_approx);

    Vector3d g_real = grad(x);

    Vector3d D = g_real - g_approx;
    bool vectors_close = check_entries<3,1>(D, h);
    EXPECT_TRUE(vectors_close);
}

TEST(OptimizeTestSuite, testApproxHessian) {
    // We can't use a value that's too small, since we're dividing by h^2 and
    // we end up losing precision.
    double h = 1e-4;
    Vector3d x = Vector3d::Ones();

    Matrix3d H_approx;
    mm::approx_hessian<3>(f, h, x, H_approx);

    Matrix3d H_real = hess(x);

    Matrix3d D = H_real - H_approx;
    bool matrices_close = check_entries<3,3>(D, h);
    EXPECT_TRUE(matrices_close);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);

    A << 1, 5, 3,
         2, 4, 7,
         4, 6, 2;
    b << 1, 2, 3;
    c = -2;

    return RUN_ALL_TESTS();
}

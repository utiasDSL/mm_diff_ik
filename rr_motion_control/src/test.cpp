#include <Eigen/Eigen>
#include <iostream>

#include "rr/kinematics.h"
#include "rr/interp.h"

using namespace Eigen;

typedef Matrix<double, 1, 1> Vector1d;

void test_interp() {
    Vector1d x1; x1 << 4;
    Vector1d x2; x2 << 10;
    Vector1d dx1; dx1 << 1;
    Vector1d dx2; dx2 << 13;

    rr::CubicInterp<1> interp(1, 2, x1, x2, dx1, dx2);

    Vector1d x, dx;

    interp.sample(1.5, x, dx);

    std::cout << x << " " << dx << std::endl;
}

int main() {
    test_interp();
    return 0;
}

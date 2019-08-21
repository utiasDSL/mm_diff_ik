#include <gtest/gtest.h>
#include <math.h>

#include "mm_math_util/wrap.h"


using namespace mm;


static const double EPS = 1e-8;


TEST(WrapTestSuite, testWrap) {
    double a1, a2;

    // Angles in [-pi, pi) should be unchanged
    a1 = M_PI_2;
    a2 = wrap_to_pi(a1);
    EXPECT_TRUE(fabs(a2 - a1) < EPS);

    a1 = -M_PI_2;
    a2 = wrap_to_pi(a1);
    EXPECT_TRUE(fabs(a2 - a1) < EPS);

    // Angles outside should get wrapped.
    a1 = 1.5*M_PI;
    a2 = wrap_to_pi(a1);
    EXPECT_TRUE(fabs(a2 + M_PI_2) < EPS);

    a1 = -1.5*M_PI;
    a2 = wrap_to_pi(a1);
    EXPECT_TRUE(fabs(a2 - M_PI_2) < EPS);

    // Handle large displacements.
    a1 = 10*M_PI;
    a2 = wrap_to_pi(a1);
    EXPECT_TRUE(fabs(a2) < EPS);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

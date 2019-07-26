#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>

using namespace Eigen;

namespace rr {
    template <unsigned int N>
    class CubicInterp {
        public:
        typedef Matrix<double, N, 1> VectorNd;
        typedef Matrix<double, 4, 1> Vector4d;
        typedef Matrix<double, 4, 4> Matrix4d;
        typedef Matrix<double, 4, N> Matrix4Nd;

        CubicInterp() {
            C = Matrix4Nd::Zero();
        }

        void interpolate(double t1, double t2, VectorNd& x1, VectorNd& x2,
                         VectorNd& dx1, VectorNd& dx2) {
            this->t1 = t1;
            this->t2 = t2;

            Matrix4d A;
            A << t1*t1*t1, t1*t1, t1, 1,
                 t2*t2*t2, t2*t2, t2, 1,
                 3*t1*t1,  2*t1,  1,  0,
                 3*t2*t2,  2*t2,  1,  0;

            Matrix4Nd B;
            B << x1, x2, dx1, dx2;

            // Solve the system AC=B to get the coefficients of the cubic
            // polynomials.
            C = A.colPivHouseholderQr().solve(B);
        }

        bool sample(const double t, VectorNd& x, VectorNd& dx) {
            Vector4d T, dT;
            T  << t*t*t, t*t, t, 1;
            dT << 3*t*t, 2*t, 1, 0;
            x = C.transpose() * T;
            dx = C.transpose() * dT;

            // true if t falls within the interpolation range, false otherwise
            return t >= t1 && t <= t2;
        }

        private:

        // coefficients
        Matrix4Nd C;

        // start and end times
        double t1, t2;
    };
}

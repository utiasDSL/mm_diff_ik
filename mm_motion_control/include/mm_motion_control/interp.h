#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>


namespace mm {

template <unsigned int N>
class CubicInterp {
    public:
        typedef Eigen::Matrix<double, N, 1> VectorNd;
        typedef Eigen::Matrix<double, 4, 1> Vector4d;
        typedef Eigen::Matrix<double, 4, 4> Matrix4d;
        typedef Eigen::Matrix<double, 4, N> Matrix4Nd;

        CubicInterp() {
            C = Matrix4Nd::Zero();
        }

        void interpolate(double t1, double t2, VectorNd& x1, VectorNd& x2,
                         VectorNd& dx1, VectorNd& dx2) {
            this->t1 = t1;
            this->t2 = t2;

            this->x1 = x1;
            this->x2 = x2;
            this->dx1 = dx1;
            this->dx2 = dx2;

            Matrix4d A;
            A << t1*t1*t1, t1*t1, t1, 1,
                 t2*t2*t2, t2*t2, t2, 1,
                 3*t1*t1,  2*t1,  1,  0,
                 3*t2*t2,  2*t2,  1,  0;

            Matrix4Nd B;
            B << x1.transpose(),
                 x2.transpose(),
                 dx1.transpose(),
                 dx2.transpose();

            // Solve the system AC=B to get the coefficients of the cubic
            // polynomials.
            C = A.colPivHouseholderQr().solve(B);
        }

        // Sample the interpolated trajectory at time t.
        bool sample(const double t, VectorNd& x, VectorNd& dx) {
            Vector4d T, dT;
            T  << t*t*t, t*t, t, 1;
            dT << 3*t*t, 2*t, 1, 0;
            x = C.transpose() * T;
            dx = C.transpose() * dT;

            // true if t falls within the interpolation range, false otherwise
            return t >= t1 && t <= t2;
        }

        // Last point in the range.
        void last(VectorNd& x, VectorNd& dx) {
            x = x2;
            dx = dx2;
        }

    private:
        // Coefficients of interpolated 3rd-order polynomials.
        Matrix4Nd C;

        // Start and end positions and velocities.
        VectorNd x1, x2, dx1, dx2;

        // start and end times
        double t1, t2;
}; // class CubicInterp


// Linear spherical interpolation between quaternions.
class QuaternionInterp {
    public:
        QuaternionInterp() {}

        void interpolate(double t1, double t2, tf::Quaternion& q1,
                         tf::Quaternion& q2) {
            this->t1 = t1;
            this->t2 = t2;

            this->q1 = q1;
            this->q2 = q2;
        }

        bool sample(const double t, tf::Quaternion& q) {
            double a = (t - t1) / (t2 - t1);
            q = q1.slerp(q2, a);
            return t >= t1 && t <= t2;
        }

    private:
        tf::Quaternion q1, q2;

        double t1, t2;

}; // class QuaternionInterp

} // namespace mm

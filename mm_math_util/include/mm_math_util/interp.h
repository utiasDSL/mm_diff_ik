#pragma once

#include <Eigen/Eigen>


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
            t1 = 0;
            t2 = 0;
        }

        void interpolate(double t1, double t2, VectorNd& x1, VectorNd& x2,
                         VectorNd& dx1, VectorNd& dx2) {
            this->t1 = t1;
            this->t2 = t2;
            double dt = t2 - t1;

            this->x1 = x1;
            this->x2 = x2;
            this->dx1 = dx1;
            this->dx2 = dx2;

            // Constraints on the cubic polynomial x(t) = c1*t^3 + c2*t^2 + c3*t + c4:
            //   x(0) = x1
            //   x(1) = x2
            //   dx(0) = dx1
            //   dx(1) = dx2
            // where dx is the derivative of x
            Matrix4d A;
            A << 0, 0, 0, 1,
                 dt*dt*dt, dt*dt, dt, 1,
                 0,  0,  1,  0,
                 3*dt*dt,  2*dt,  1,  0;

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
            double ta = t - t1;

            Vector4d T, dT;
            T  << ta*ta*ta, ta*ta, ta, 1;
            dT << 3*ta*ta, 2*ta, 1, 0;

            x = C.transpose() * T;
            dx = C.transpose() * dT;

            return inrange(t);
        }

        // Last point in the range.
        void last(VectorNd& x, VectorNd& dx) {
            x = x2;
            dx = dx2;
        }

        // True if t falls within the interpolation range, false otherwise
        bool inrange(const double t) {
            return t >= t1 && t <= t2;
        }

    private:
        // Coefficients of interpolated 3rd-order polynomials.
        Matrix4Nd C;

        // Start and end positions and velocities.
        VectorNd x1, x2, dx1, dx2;

        // Start and end times
        double t1, t2;
}; // class CubicInterp


template <unsigned int N>
class QuinticInterp {
    public:
        typedef Eigen::Matrix<double, N, 1> VectorNd;
        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        typedef Eigen::Matrix<double, 6, 6> Matrix6d;
        typedef Eigen::Matrix<double, 6, N> Matrix6Nd;

        QuinticInterp() {
            C = Matrix6Nd::Zero();
            t1 = 0;
            t2 = 0;
        }

        void interpolate(double t1, double t2, VectorNd& x1, VectorNd& x2,
                         VectorNd& dx1, VectorNd& dx2, VectorNd& ddx1,
                         VectorNd& ddx2) {
            this->t1 = t1;
            this->t2 = t2;
            double dt = t2 - t1;

            this->x1 = x1;
            this->x2 = x2;
            this->dx1 = dx1;
            this->dx2 = dx2;
            this->ddx1 = ddx1;
            this->ddx2 = ddx2;

            double dt2 = dt*dt;
            double dt3 = dt2*dt;
            double dt4 = dt3*dt;
            double dt5 = dt4*dt;

            Matrix6d A;
            A << 0, 0, 0, 0, 0, 1,
                 dt5, dt4, dt3, dt2, dt, 1,
                 0,  0,  0, 0, 1,  0,
                 5*dt4, 4*dt3, 3*dt2,  2*dt,  1,  0,
                 0, 0, 0, 2, 0, 0,
                 20*dt3, 12*dt2, 6*dt, 2, 0, 0;

            Matrix6Nd B;
            B << x1.transpose(),
                 x2.transpose(),
                 dx1.transpose(),
                 dx2.transpose(),
                 ddx1.transpose(),
                 ddx2.transpose();

            // Solve the system AC=B to get the coefficients of the cubic
            // polynomials.
            C = A.colPivHouseholderQr().solve(B);
        }

        // Sample the interpolated trajectory at time t.
        bool sample(const double t, VectorNd& x, VectorNd& dx, VectorNd& ddx) {
            double ta = t - t1;

            double ta2 = ta*ta;
            double ta3 = ta2*ta;
            double ta4 = ta3*ta;
            double ta5 = ta4*ta;

            Vector6d T, dT, ddT;
            T  << ta5, ta4, ta3, ta2, ta, 1;
            dT << 5*ta4, 4*ta3, 3*ta2,  2*ta,  1,  0;
            ddT << 20*ta3, 12*ta2, 6*ta, 2, 0, 0;

            x = C.transpose() * T;
            dx = C.transpose() * dT;
            ddx = C.transpose() * ddT;

            return inrange(t);
        }

        // Last point in the range.
        void last(VectorNd& x, VectorNd& dx, VectorNd& ddx) {
            x = x2;
            dx = dx2;
            ddx = ddx2;
        }

        // True if t falls within the interpolation range, false otherwise
        bool inrange(const double t) {
            return t >= t1 && t <= t2;
        }

    private:
        // Coefficients of interpolated 3rd-order polynomials.
        Matrix6Nd C;

        // Start and end positions, velocities, and accelerations.
        VectorNd x1, x2, dx1, dx2, ddx1, ddx2;

        // Start and end times
        double t1, t2;
}; // class CubicInterp


// Linear spherical interpolation between quaternions.
class QuaternionInterp {
    public:
        QuaternionInterp() {}

        void interpolate(double t1, double t2, Eigen::Quaterniond& q1,
                         Eigen::Quaterniond& q2) {
            this->t1 = t1;
            this->t2 = t2;

            this->q1 = q1;
            this->q2 = q2;
        }

        bool sample(const double t, Eigen::Quaterniond& q) {
            double a = (t - t1) / (t2 - t1);
            q = q1.slerp(a, q2);
            return t >= t1 && t <= t2;
        }

    private:
        Eigen::Quaterniond q1, q2;

        double t1, t2;

}; // class QuaternionInterp


} // namespace mm

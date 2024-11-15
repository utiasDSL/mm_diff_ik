#pragma once

#include <Eigen/Eigen>

#include "mm_math_util/rotation.h"

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

  void interpolate(double t1,
                   double t2,
                   VectorNd& x1,
                   VectorNd& x2,
                   VectorNd& dx1,
                   VectorNd& dx2) {
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
    A << 0, 0, 0, 1, dt * dt * dt, dt * dt, dt, 1, 0, 0, 1, 0, 3 * dt * dt,
        2 * dt, 1, 0;

    Matrix4Nd B;
    B << x1.transpose(), x2.transpose(), dx1.transpose(), dx2.transpose();

    // Solve the system AC=B to get the coefficients of the cubic
    // polynomials.
    C = A.colPivHouseholderQr().solve(B);
  }

  // Sample the interpolated trajectory at time t.
  bool sample(const double t, VectorNd& x, VectorNd& dx) {
    double ta = t - t1;

    Vector4d T, dT;
    T << ta * ta * ta, ta * ta, ta, 1;
    dT << 3 * ta * ta, 2 * ta, 1, 0;

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
  bool inrange(const double t) { return t >= t1 && t <= t2; }

 private:
  // Coefficients of interpolated 3rd-order polynomials.
  Matrix4Nd C;

  // Start and end positions and velocities.
  VectorNd x1, x2, dx1, dx2;

  // Start and end times
  double t1, t2;
};  // class CubicInterp

template <unsigned int N>
class QuinticInterp {
 public:
  typedef Eigen::Matrix<double, N, 1> VectorNd;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 6, N> Matrix6Nd;

  QuinticInterp() {
    C = Matrix6Nd::Zero();
    dt = 0;
  }

  void interpolate(double dt,
                   const VectorNd& x1,
                   const VectorNd& x2,
                   const VectorNd& dx1,
                   const VectorNd& dx2,
                   const VectorNd& ddx1,
                   const VectorNd& ddx2) {
    this->dt = dt;

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    double dt5 = dt4 * dt;

    // x(t) = c5*t^5 + c4*t^4 + c3*t^3 + c2*t^2 + c1*t + c0
    Matrix6d A;
    A << 0, 0, 0, 0, 0, 1, dt5, dt4, dt3, dt2, dt, 1, 0, 0, 0, 0, 1, 0, 5 * dt4,
        4 * dt3, 3 * dt2, 2 * dt, 1, 0, 0, 0, 0, 2, 0, 0, 20 * dt3, 12 * dt2,
        6 * dt, 2, 0, 0;

    Matrix6Nd B;
    B << x1.transpose(), x2.transpose(), dx1.transpose(), dx2.transpose(),
        ddx1.transpose(), ddx2.transpose();

    // Solve the system AC=B to get the coefficients of the cubic
    // polynomials.
    C = A.colPivHouseholderQr().solve(B);
  }

  // Sample the interpolated trajectory at time t.
  bool sample(const double t, VectorNd& x, VectorNd& dx, VectorNd& ddx) {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    Vector6d T, dT, ddT;
    T << t5, t4, t3, t2, t, 1;
    dT << 5 * t4, 4 * t3, 3 * t2, 2 * t, 1, 0;
    ddT << 20 * t3, 12 * t2, 6 * t, 2, 0, 0;

    x = C.transpose() * T;
    dx = C.transpose() * dT;
    ddx = C.transpose() * ddT;

    return inrange(t);
  }

  // True if t falls within the interpolation range, false otherwise
  bool inrange(const double t) { return t >= 0 && t <= dt; }

 private:
  // Coefficients of interpolated 3rd-order polynomials.
  Matrix6Nd C;

  // Duration
  double dt;
};  // class QuinticInterp

// Linear spherical interpolation between quaternions.
class QuaternionInterp {
 public:
  QuaternionInterp() {}

  void interpolate(double dt,
                   const Eigen::Quaterniond& q1,
                   const Eigen::Quaterniond& q2) {
    this->dt = dt;
    this->q1 = q1;
    this->q2 = q2;
  }

  bool sample(const double t, Eigen::Quaterniond& q) {
    double s = t / dt;
    q = q1.slerp(s, q2);
    return t >= 0 && t <= dt;
  }

 private:
  Eigen::Quaterniond q1, q2;

  double dt;

};  // class QuaternionInterp

// Spherical linear interpolation between 2D unit vectors a and b.
inline Eigen::Vector2d slerp2d(const Eigen::Vector2d& a,
                               const Eigen::Vector2d& b,
                               const double t) {
  Eigen::Vector2d an = a.normalized();
  Eigen::Vector2d bn = b.normalized();

  const double eps = 1e-8;
  const double eps1 = 1 - eps;

  // If either vector is zero, return the other vector. If both are zero,
  // this leads to a zero vector being returned.
  if (an.norm() < eps) {
    return bn;
  } else if (bn.norm() < eps) {
    return an;
  }

  const double dot = an.dot(bn);
  const double angle = std::acos(dot);

  if (dot >= eps1) {
    // Unit vectors are the same: just return one of them.
    return an;
  } else if (dot <= -eps1) {
    // Unit vectors are opposite: slerp formula has numeric issues, so use
    // rotation matrix.
    Eigen::Matrix2d R = rotation2d(t * angle);
    return R * an;
  }

  Eigen::Vector2d c =
      (std::sin((1 - t) * angle) * an + std::sin(t * angle) * bn) /
      std::sin(angle);
  return c.normalized();
}

}  // namespace mm

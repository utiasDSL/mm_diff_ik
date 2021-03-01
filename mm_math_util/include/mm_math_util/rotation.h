#pragma once

#include <math.h>
#include <Eigen/Eigen>

namespace mm {

// Get 2D rotation matrix corresponding to angle.
inline Eigen::Matrix2d rotation2d(const double angle) {
  double c = std::cos(angle);
  double s = std::sin(angle);
  Eigen::Matrix2d R;
  R << c, -s, s, c;
  return R;
}

inline Eigen::Matrix2d rotation2d_derivative(const double angle) {
  Eigen::Matrix2d R = rotation2d(angle);
  Eigen::Matrix2d S;
  S << 0, -1, 1, 0; // skew matrix
  return S * R;
}

}  // namespace mm

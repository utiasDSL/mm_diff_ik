#pragma once

#include <Eigen/Eigen>

namespace mm {

// Form cross product matrix of vector v.
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<  0,    -v(2),  v(1),
          v(2),  0,    -v(0),
         -v(1),  v(0),  0;
    return m;
}

} // namespace mm

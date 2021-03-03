#pragma once

#include <Eigen/Eigen>

#include <mm_math_util/interp.h>

#include <mm_control/cartesian/point.h>

namespace mm {

class CartesianSplineSegment {
 public:
  // Initialize and interpolate between the provided points.
  CartesianSplineSegment(const CartesianTrajectoryPoint& a,
                         const CartesianTrajectoryPoint& b);

  // Sample the spline. The given time is relative to the start of the
  // trajectory.
  bool sample(double time, CartesianTrajectoryPoint& point);

  // time is relative to the start of the trajectory
  double start_time;
  double duration;

 private:
  // Quintic interpolation for the translational component.
  QuinticInterp<3> pos_interp;

  // Spherical linear interpolation (slerp) for the angular component. Angular
  // velocity and acceleration are assumed to be constant.
  QuaternionInterp quat_interp;
  Eigen::Vector3d angular_vel;
  Eigen::Vector3d angular_acc;
};

}  // namespace mm

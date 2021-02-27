#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_control/cartesian/types.h>
#include <mm_kinematics/spatial.h>
#include <mm_math_util/interp.h>

namespace mm {

class CartesianSplineSegment {
 public:
  // Initialize and interpolate between the provided points.
  CartesianSplineSegment(CartesianTrajectoryPoint& a,
                         CartesianTrajectoryPoint& b);

  // Sample the spline. The given time is relative to the start of the
  // trajectory.
  bool sample(double time, CartesianPosVelAcc& state);

  // time is relative to the start of the trajectory
  double time;
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

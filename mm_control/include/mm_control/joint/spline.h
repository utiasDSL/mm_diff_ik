#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_control/joint/types.h>
#include <mm_math_util/interp.h>

namespace mm {

class JointSplineSegment {
 public:
  // Initialize and interpolate between the provided points.
  CartesianSplineSegment(const JointTrajectoryPoint& a,
                         const JointTrajectoryPoint& b);

  // Sample the spline. The given time is relative to the start of the
  // trajectory.
  bool sample(double time, JointTrajectoryState& state);

  // time is relative to the start of the trajectory
  double time;
  double duration;

 private:
  QuinticInterp<NUM_JOINTS> interp;
};

}  // namespace mm

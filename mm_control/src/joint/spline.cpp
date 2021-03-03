#include "mm_control/joint/spline.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

namespace mm {

JointSplineSegment::JointSplineSegment(const JointTrajectoryPoint& a,
                                       const JointTrajectoryPoint& b) {
  time = a.time;
  duration = b.time - a.time;

  interp.interpolate(a.time, b.time, a.positions, b.positions, a.velocities,
                     b.velocities, a.accelerations, b.accelerations);
}

bool JointSplineSegment::sample(double time, JointTrajectoryPoint& point) {
  interp.sample(time, point.positions, point.velocities, point.accelerations);
  return true;
}

}  // namespace mm

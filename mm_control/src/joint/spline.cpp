#include "mm_control/joint/spline.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

namespace mm {

JointSplineSegment::JointSplineSegment(const JointTrajectoryPoint& a,
                                       const JointTrajectoryPoint& b) {
  start_time = a.time;
  duration = b.time - a.time;

  interp.interpolate(duration, a.positions, b.positions, a.velocities,
                     b.velocities, a.accelerations, b.accelerations);
}

bool JointSplineSegment::sample(double time, JointTrajectoryPoint& point) {
  interp.sample(time - start_time, point.positions, point.velocities, point.accelerations);
  return true;
}

}  // namespace mm

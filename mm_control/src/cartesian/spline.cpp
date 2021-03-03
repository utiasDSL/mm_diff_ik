#include "mm_control/cartesian/spline.h"

#include <mm_control/cartesian/point.h>

namespace mm {

CartesianSplineSegment::CartesianSplineSegment(
    const CartesianTrajectoryPoint& a, const CartesianTrajectoryPoint& b) {
  time = a.time;
  duration = b.time - a.time;

  // TODO I would prefer these APIs to just take a duration
  // interpolation translation
  pos_interp.interpolate(a.time, b.time, a.pose.position, b.pose.position,
                         a.twist.linear, b.twist.linear, a.acceleration.linear,
                         b.acceleration.linear);

  // interpolate orientation
  quat_interp.interpolate(a.time, b.time, a.pose.orientation,
                          b.pose.orientation);
  angular_vel = a.twist.angular;
  angular_acc = a.acceleration.angular;
}

bool CartesianSplineSegment::sample(double time,
                                    CartesianTrajectoryPoint& point) {
  pos_interp.sample(time, point.pose.position, point.twist.linear,
                    point.acceleration.linear);

  quat_interp.sample(time, point.pose.orientation);
  point.twist.angular = angular_vel;
  point.acceleration.angular = angular_acc;

  point.time = time;

  return true;
}

}  // namespace mm

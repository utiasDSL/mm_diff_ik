#include "mm_control/cartesian/spline.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

namespace mm {

CartesianSplineSegment::CartesianSplineSegment(
    const CartesianTrajectoryPoint& a, const CartesianTrajectoryPoint& b) {
  time = a.time;
  duration = b.time - a.time;

  // TODO I would prefer these APIs to just take a duration
  // interpolation translation
  pos_interp.interpolate(a.time, b.time, a.state.pose.position,
                         b.state.pose.position, a.state.twist.linear,
                         b.state.twist.linear, a.state.acceleration.linear,
                         b.state.acceleration.linear);

  // interpolate orientation
  quat_interp.interpolate(a.time, b.time, a.state.pose.orientation,
                          b.state.pose.orientation);
  angular_vel = a.state.twist.angular;
  angular_acc = a.state.acceleration.angular;
}

bool CartesianSplineSegment::sample(double time, CartesianPosVelAcc& state) {
  pos_interp.sample(time, state.pose.position, state.twist.linear,
                    state.acceleration.linear);

  quat_interp.sample(time, state.pose.orientation);
  state.twist.angular = angular_vel;
  state.acceleration.angular = angular_acc;
  return true;
}

}  // namespace mm

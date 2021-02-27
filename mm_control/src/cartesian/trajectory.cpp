#include "mm_control/cartesian/trajectory.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_kinematics/spatial.h>
#include <mm_math_util/interp.h>
#include <mm_msgs/CartesianState.h>
#include <mm_msgs/CartesianTrajectory.h>
#include <mm_msgs/CartesianTrajectoryPoint.h>
#include <mm_msgs/conversions.h>
#include <mm_control/util/messages.h>

namespace mm {

CartesianTrajectoryPoint cartesian_waypoint_from_message(
    const mm_msgs::CartesianTrajectoryPoint& msg) {
  CartesianTrajectoryPoint waypoint;

  // TODO refactor this
  waypoint.time = msg.time.toSec();
  waypoint.state.pose.position = point_msg_to_eigen(msg.state.pose.position);
  waypoint.state.pose.orientation = quat_msg_to_eigen(msg.state.pose.orientation);
  waypoint.state.twist.linear
      << msg.state.twist.linear.x, msg.state.twist.linear.y,
      msg.state.twist.linear.z;
  waypoint.state.twist.angular << msg.state.twist.angular.x, msg.state.twist.angular.y,
      msg.state.twist.angular.z;
  waypoint.state.acc.linear << msg.state.acceleration.linear.x, msg.state.acceleration.linear.y,
      msg.state.acceleration.linear.z;
  waypoint.state.acc.angular << msg.state.acceleration.angular.x,
      msg.state.acceleration.angular.y, msg.state.acceleration.angular.z;

  return waypoint;
}

// TODO maybe just one init function with overloads? more consistent
bool CartesianTrajectory::from_message(mm_msgs::CartesianTrajectory msg) {
  std::vector<CartesianTrajectoryPoint> waypoints;
  for (auto point : msg.points) {
    auto waypoint = cartesian_waypoint_from_message(point);
    waypoints.push_back(waypoint);
  }
  return from_waypoints(waypoints);
}

bool CartesianTrajectory::from_waypoints(
    std::vector<CartesianTrajectoryPoint> points) {
  for (int i = 0; i < points.size() - 1; ++i) {
    CartesianSplineSegment segment(points[i], points[i + 1]);
    segments.push_back(segment);
  }
  first = points[0].state;
  last = points.back().state;
  duration = points.back().time - points[0].time;
  stationary = false;
  return true;
}

bool CartesianTrajectory::from_point(Pose pose) {
  // First and last points are the same, and stationary at the given pose.
  first.pose = pose;
  last = first;

  duration = 0;
  stationary = true;
  return true;
}

bool CartesianTrajectory::start(const ros::Time& time) { start_time = time.toSec(); }

bool CartesianTrajectory::sample(const ros::Time& time,
                                 CartesianPosVelAcc& state) {
  if (done(time)) {
    state = last;
    return false;
  }

  // Update current waypoints
  // TODO I think edge case is handled by call to done above?
  double t = time.toSec();
  while (t > start_time + segments[index].time + segments[index].duration) {
    index++;
  }

  return segments[index].sample(t - start_time, state);
}

bool CartesianTrajectory::done(const ros::Time& time) {
  // Stationary trajectories don't end
  if (stationary) {
    return false;
  }
  // Other trajectories end once their duration is expired
  return time.toSec() > start_time + duration;
}

CartesianSplineSegment::CartesianSplineSegment(CartesianTrajectoryPoint& a,
                                               CartesianTrajectoryPoint& b) {
  time = a.time;
  duration = b.time - a.time;

  // TODO I would prefer these APIs to just take a duration
  // interpolation translation
  pos_interp.interpolate(a.time, b.time, a.state.pose.position,
                         b.state.pose.position, a.state.twist.linear,
                         b.state.twist.linear, a.state.acc.linear,
                         b.state.acc.linear);

  // interpolate orientation
  quat_interp.interpolate(a.time, b.time, a.state.pose.orientation,
                          b.state.pose.orientation);
  angular_vel = a.state.twist.angular;
  angular_acc = a.state.acc.angular;
}

bool CartesianSplineSegment::sample(double time, CartesianPosVelAcc& state) {
  // For interpolation we only care about time since the first point on this
  // segment.
  // double dt = time - this->time;
  pos_interp.sample(time, state.pose.position, state.twist.linear, state.acc.linear);

  quat_interp.sample(time, state.pose.orientation);
  state.twist.angular = angular_vel;
  state.acc.angular = angular_acc;
  return true;
}

}  // namespace mm

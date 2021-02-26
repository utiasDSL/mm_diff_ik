#include "mm_control/cartesian/trajectory.h"

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>

#include <mm_math_util/interp.h>
#include <mm_msgs/PoseTrajectory.h>
#include <mm_msgs/PoseTrajectoryPoint.h>
#include <mm_msgs/conversions.h>

namespace mm {

CartesianTrajectory::CartesianTrajectory() {}

bool CartesianTrajectory::from_message(mm_msgs::CartesianTrajectory msg) {
  // TODO need convert from message to waypoint
  std::vector<CartesianTrajectoryPoint> waypoints;
  for (auto point : msg.points) {
    auto waypoint = 0;
    waypoints.push_back(waypoint);
  }
  return from_waypoints(waypoints);
}

bool CartesianTrajectory::from_waypoints(
    std::vector<CartesianTrajectoryPoint> points) {
  for (int i = 0; i < waypoints.size() - 1; ++i) {
    auto segment = CartesianSplineSegment(waypoints[i], waypoints[i + 1]);
    segments.push_back(segment);
  }
  first = points[0].state;
  last = points.back().state;
  duration = points.back().time - points[0].time;
  stationary = false;
  return true;
}

bool CartesianTrajectory::from_point(Eigen::Affine3d pose) {
  // First and last points are the same, and stationary at the given pose.
  first.state.pose = pose;
  first.state.twist = Vector6d::Zero();
  first.state.acc = Vector6d::Zero();
  last = first;

  duration = 0;
  stationary = true;
  return true;
}

bool CartesianTrajectory::start(ros::Time time) { start_time = time.toSec(); }

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

}  // namespace mm

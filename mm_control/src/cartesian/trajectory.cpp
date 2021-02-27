#include "mm_control/cartesian/trajectory.h"

#include <ros/ros.h>

#include <mm_kinematics/spatial.h>
#include <mm_msgs/CartesianState.h>
#include <mm_msgs/CartesianTrajectory.h>
#include <mm_msgs/CartesianTrajectoryPoint.h>
#include <mm_msgs/conversions.h>

namespace mm {

bool CartesianTrajectory::init(mm_msgs::CartesianTrajectory msg) {
  std::vector<CartesianTrajectoryPoint> waypoints;

  for (auto point : msg.points) {
    CartesianTrajectoryPoint waypoint;
    waypoint.time = point.time.toSec();
    waypoint.state = cartesian_state_to_eigen(point.state);
    waypoints.push_back(waypoint);
  }

  return init(waypoints);
}

bool CartesianTrajectory::init(std::vector<CartesianTrajectoryPoint> points) {
  for (int i = 0; i < points.size() - 1; ++i) {
    CartesianSplineSegment segment(points[i], points[i + 1]);
    segments.push_back(segment);
  }

  first = points[0].state;
  last = points.back().state;

  duration = points.back().time - points[0].time;
  is_finite = true;
  is_initialized = true;
  return true;
}

bool CartesianTrajectory::init(Pose pose) {
  // First and last points are the same, and stationary at the given pose.
  first.pose = pose;
  last = first;

  duration = 0;
  is_finite = false;
  is_initialized = true;
  return true;
}

bool CartesianTrajectory::sample(const ros::Time& time,
                                 CartesianPosVelAcc& state) {
  if (!started() || finished(time)) {
    return false;
  }

  // Update current segment
  double t = time.toSec();
  while (t > start_time + segments[index].time + segments[index].duration) {
    index++;
  }

  return segments[index].sample(t - start_time, state);
}

}  // namespace mm

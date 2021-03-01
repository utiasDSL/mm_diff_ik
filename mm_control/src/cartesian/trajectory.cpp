#include "mm_control/cartesian/trajectory.h"

#include <ros/ros.h>

#include <mm_kinematics/spatial.h>
#include <mm_msgs/CartesianState.h>
#include <mm_msgs/CartesianTrajectory.h>
#include <mm_msgs/CartesianTrajectoryPoint.h>
#include <mm_msgs/conversions.h>

namespace mm {

bool CartesianTrajectory::init(const mm_msgs::CartesianTrajectory& msg) {
  std::vector<CartesianTrajectoryPoint> waypoints;

  for (auto point : msg.points) {
    CartesianTrajectoryPoint waypoint;
    waypoint.time = point.time.toSec();
    waypoint.state = cartesian_state_to_eigen(point.state);
    waypoints.push_back(waypoint);
  }

  return init(waypoints);
}

bool CartesianTrajectory::init(
    const std::vector<CartesianTrajectoryPoint>& points) {
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

bool CartesianTrajectory::init(const Pose& pose) {
  // First and last points are the same, and stationary at the given pose.
  first.pose = pose;
  last = first;

  duration = 0;
  is_finite = false;
  is_initialized = true;
  return true;
}

bool CartesianTrajectory::sample(const ros::Time& now,
                                 CartesianPosVelAcc& state) {
  if (!started() || finished(now)) {
    // TODO should this be here? It is undefined behaviour, but am I relying on
    // it?
    state = last;
    return false;
  }

  double t = now.toSec();

  // If the trajectory is past its end time but infinite, return the last
  // point.
  if (!is_finite && t >= start_time + duration) {
    state = last;
    return true;
  }

  // Finite, active trajectory: update current segment
  while (t > start_time + segments[index].time + segments[index].duration) {
    index++;
  }
  return segments[index].sample(t - start_time, state);
}

bool CartesianTrajectory::sample(const ros::Time& now,
                                 const std::vector<ros::Time>& times,
                                 std::vector<CartesianPosVelAcc>& states) {
  if (!started() || finished(now)) {
    return false;
  }

  // Update the current index based on the provided current time.
  double t = now.toSec();
  while (t > start_time + segments[index].time + segments[index].duration) {
    index++;
  }

  // Sample at all of the sample times, using the dummy variable i to avoid
  // modifying the real index.
  size_t i = index;
  for (auto time : times) {
    if (finished(time)) {
      states.push_back(last);
      continue;
    }

    t = time.toSec();
    if (!is_finite && t >= start_time + duration) {
      states.push_back(last);
      continue;
    }

    while (t > start_time + segments[i].time + segments[i].duration) {
      i++;
    }

    CartesianPosVelAcc state;
    segments[i].sample(t - start_time, state);
    states.push_back(state);
  }
  return true;
}

}  // namespace mm

#include "mm_control/joint/trajectory.h"

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace mm {

bool JointTrajectory::init(const trajectory_msgs::JointTrajectory& msg) {
  std::vector<JointTrajectoryPoint> waypoints;

  for (auto point : msg.points) {
    waypoints.push_back(JointTrajectoryPoint(point));
  }

  return init(waypoints);
}

bool JointTrajectory::init(const std::vector<JointTrajectoryPoint>& points) {
  for (int i = 0; i < points.size() - 1; ++i) {
    JointSplineSegment segment(points[i], points[i + 1]);
    segments.push_back(segment);
  }

  first = points[0];
  last = points.back();

  duration = last.time - first.time;
  is_finite = true;
  is_initialized = true;
  return true;
}

bool JointTrajectory::init(const trajectory_msgs::JointTrajectoryPoint& msg) {
  // First and last points are the same, and stationary at the given pose.
  first.positions = JointVector(msg.positions.data());
  last = first;

  duration = 0;
  is_finite = false;
  is_initialized = true;
  return true;
}

bool JointTrajectory::sample(const ros::Time& now,
                             JointTrajectoryPoint& point) {
  if (!started() || finished(now)) {
    // TODO should this be here? It is undefined behaviour, but am I relying on
    // it?
    point = last;
    return false;
  }

  double t = now.toSec();

  // If the trajectory is past its end time but infinite, return the last
  // point.
  if (!is_finite && t >= start_time + duration) {
    point = last;
    return true;
  }

  // Finite, active trajectory: update current segment
  while (t > start_time + segments[index].time + segments[index].duration) {
    index++;
  }
  return segments[index].sample(t - start_time, point);
}

bool JointTrajectory::sample(const ros::Time& now,
                             const std::vector<ros::Time>& times,
                             std::vector<JointTrajectoryPoint>& points) {
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
      points.push_back(last);
      continue;
    }

    t = time.toSec();
    if (!is_finite && t >= start_time + duration) {
      points.push_back(last);
      continue;
    }

    while (t > start_time + segments[i].time + segments[i].duration) {
      i++;
    }

    JointTrajectoryPoint point;
    segments[i].sample(t - start_time, point);
    points.push_back(point);
  }
  return true;
}

}  // namespace mm

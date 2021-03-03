#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_control/joint/point.h>
#include <mm_control/joint/spline.h>
#include <mm_control/trajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace mm {

// Trajectory in joint space.
class JointTrajectory : public Trajectory {
 public:
  JointTrajectory() {}

  // Initialize from a list of waypoints.
  bool init(const std::vector<JointTrajectoryPoint>& points);

  // Initialize from a ROS message.
  bool init(const trajectory_msgs::JointTrajectory& msg);

  // Initialize from a single setpoint.
  bool init(const trajectory_msgs::JointTrajectoryPoint& msg);

  // Sample the trajectory at the given time `now`. Returns false if the
  // trajectory
  // is sampled in an invalid state, in which case the `state` variable should
  // not be trusted. Trajectories are assumed to be sampled at monotonically
  // increasing times.
  bool sample(const ros::Time& now, JointTrajectoryPoint& point);

  // Sample the trajectory at `times` in the after the given time `now`. The
  // `times` are expected to be monotonically increasing.
  bool sample(const ros::Time& now,
              const std::vector<ros::Time>& times,
              std::vector<JointTrajectoryPoint>& points);

 protected:
  // First and last points in the trajectory.
  JointTrajectoryPoint first, last;

  // Spline segments making up the trajectory
  std::vector<JointSplineSegment> segments;
};

}  // namespace mm

#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace mm {

// Waypoint for a joint space trajectory.
struct JointTrajectoryPoint {
  JointTrajectoryPoint();

  // Initialize from a ROS message.
  JointTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& msg);

  // Convert to a ROS message.
  void message(trajectory_msgs::JointTrajectoryPoint& msg);

  double time;
  JointVector positions;
  JointVector velocities;
  JointVector accelerations;
};

}  // namespace mm

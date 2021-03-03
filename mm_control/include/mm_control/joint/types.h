#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace mm {

struct JointTrajectoryState {
  JointVector positions;
  JointVector velocities;
  JointVector accelerations;
};

struct JointTrajectoryPoint {
  double time;
  JointTrajectoryState state;
};

inline JointTrajectoryPoint joint_point_to_eigen(
    const trajectory_msgs::JointTrajectoryPoint& msg) {
  JointTrajectoryPoint point;
  point.positions = JointVector(msg.positions.data());
  point.velocities = JointVector(msg.velocities.data());
  point.accelerations = JointVector(msg.accelerations.data());
  return point;
}

inline void joint_point_from_eigen(const JointTrajectoryPoint& point,
                                   trajectory_msgs::JointTrajectoryPoint& msg) {
  msg.positions = std::vector<double>(
      point.positions.data(), point.positions.data() + point.positions.size());
  msg.velocities =
      std::vector<double>(point.velocities.data(),
                          point.velocities.data() + point.velocities.size());
  msg.accelerations = std::vector<double>(
      point.accelerations.data(),
      point.accelerations.data() + point.accelerations.size());
}

}  // namespace mm

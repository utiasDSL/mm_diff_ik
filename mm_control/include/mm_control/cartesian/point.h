#pragma once

#include <mm_kinematics/conversion.h>
#include <mm_kinematics/spatial.h>
#include <mm_msgs/CartesianTrajectoryPoint.h>

namespace mm {

struct CartesianTrajectoryPoint {
  CartesianTrajectoryPoint() {}

  // Initialize from a ROS message.
  CartesianTrajectoryPoint(const mm_msgs::CartesianTrajectoryPoint& msg);

  // Convert to a ROS message.
  void message(mm_msgs::CartesianTrajectoryPoint& msg);

  double time;
  Pose pose;
  Twist twist;
  Twist acceleration;
};

}  // namespace mm

#include "mm_control/joint/point.h"

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace mm {

JointTrajectoryPoint::JointTrajectoryPoint() {
  time = 0;
  positions = JointVector::Zero();
  velocities = JointVector::Zero();
  accelerations = JointVector::Zero();
}

JointTrajectoryPoint::JointTrajectoryPoint(
    const trajectory_msgs::JointTrajectoryPoint& msg) {
  time = msg.time_from_start.toSec();
  positions = JointVector(msg.positions.data());
  velocities = JointVector(msg.velocities.data());
  accelerations = JointVector(msg.accelerations.data());
}

void JointTrajectoryPoint::message(trajectory_msgs::JointTrajectoryPoint& msg) {
  msg.time_from_start = ros::Duration(time);
  msg.positions = std::vector<double>(positions.data(),
                                      positions.data() + positions.size());
  msg.velocities = std::vector<double>(velocities.data(),
                                       velocities.data() + velocities.size());
  msg.accelerations = std::vector<double>(
      accelerations.data(), accelerations.data() + accelerations.size());
}

}  // namespace mm

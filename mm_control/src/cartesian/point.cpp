#include "mm_control/cartesian/point.h"

#include <ros/ros.h>

#include <mm_kinematics/conversion.h>
#include <mm_kinematics/spatial.h>
#include <mm_msgs/CartesianTrajectoryPoint.h>

namespace mm {

CartesianTrajectoryPoint::CartesianTrajectoryPoint(
    const mm_msgs::CartesianTrajectoryPoint& msg) {
  time = msg.time.toSec();
  pose = pose_msg_to_eigen(msg.pose);
  twist = twist_msg_to_eigen(msg.twist);
  acceleration = twist_msg_to_eigen(msg.acceleration);
}

void CartesianTrajectoryPoint::message(mm_msgs::CartesianTrajectoryPoint& msg) {
  msg.time = ros::Duration(time);
  pose_msg_from_eigen(pose, msg.pose);
  twist_msg_from_eigen(twist, msg.twist);
  twist_msg_from_eigen(acceleration, msg.acceleration);
}

}  // namespace mm

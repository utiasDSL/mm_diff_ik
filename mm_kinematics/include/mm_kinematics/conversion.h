#pragma once

// Utilities for converting between Eigen-based types and ROS messages.

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <mm_kinematics/spatial.h>

namespace mm {

inline Eigen::Vector3d vec3_msg_to_eigen(const geometry_msgs::Vector3& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Vector3d point_msg_to_eigen(const geometry_msgs::Point& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Quaterniond quat_msg_to_eigen(
    const geometry_msgs::Quaternion& msg) {
  return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}

inline Pose pose_msg_to_eigen(const geometry_msgs::Pose& msg) {
  Pose pose;
  pose.position = point_msg_to_eigen(msg.position);
  pose.orientation = quat_msg_to_eigen(msg.orientation);
  return pose;
}

inline Twist twist_msg_to_eigen(const geometry_msgs::Twist& msg) {
  Twist twist;
  twist.linear = vec3_msg_to_eigen(msg.linear);
  twist.angular = vec3_msg_to_eigen(msg.angular);
  return twist;
}

inline void vec3_msg_from_eigen(const Eigen::Vector3d& v,
                                geometry_msgs::Vector3& msg) {
  msg.x = v(0);
  msg.y = v(1);
  msg.z = v(2);
}

inline void point_msg_from_eigen(const Eigen::Vector3d& v,
                                 geometry_msgs::Point& msg) {
  msg.x = v(0);
  msg.y = v(1);
  msg.z = v(2);
}

inline void quat_msg_from_eigen(const Eigen::Quaterniond& Q,
                                geometry_msgs::Quaternion& msg) {
  msg.x = Q.x();
  msg.y = Q.y();
  msg.z = Q.z();
  msg.w = Q.w();
}

inline void pose_msg_from_eigen(const Pose& P, geometry_msgs::Pose& msg) {
  point_msg_from_eigen(P.position, msg.position);
  quat_msg_from_eigen(P.orientation, msg.orientation);
}

inline void twist_msg_from_eigen(const Twist& V, geometry_msgs::Twist& msg) {
  vec3_msg_from_eigen(V.linear, msg.linear);
  vec3_msg_from_eigen(V.angular, msg.angular);
}

}  // namespace mm

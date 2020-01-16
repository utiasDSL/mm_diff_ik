#pragma once

#include <Eigen/Eigen>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>

#include <mm_msgs/PoseTrajectoryPoint.h>


namespace mm {

// Convert a PoseTrajectoryPoint message into Eigen position, quaternion,
// linear velocity, and angular velocity vectors.
// Parameters:
//   msg: PoseTrajectoryPoint message to convert
//   p: populated position
//   q: populated quaternion
//   v: populated linear velocity
//   w: populated angular velocity
void pose_traj_point_to_eigen(const mm_msgs::PoseTrajectoryPoint& msg,
                              Eigen::Vector3d& p, Eigen::Quaterniond& q,
                              Eigen::Vector3d& v, Eigen::Vector3d& w);


// Convert a vector of joint velocities to ROS messages for base and arm.
// Parameters:
//   dq: Vector of joint velocities.
//   traj_arm: Joint trajectory message for the arm.
//   twist_base: Twist message for the base.
void joint_speed_msgs(const Eigen::Matrix<double, 9, 1>& dq,
                      trajectory_msgs::JointTrajectory& traj_arm,
                      geometry_msgs::Twist& twist_base);

} // namespace mm

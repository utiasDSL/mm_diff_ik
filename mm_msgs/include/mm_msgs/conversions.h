#pragma once

#include <Eigen/Eigen>

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

} // namespace mm

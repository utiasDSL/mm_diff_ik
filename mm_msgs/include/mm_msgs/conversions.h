#pragma once

#include <Eigen/Eigen>
#include <mm_msgs/PoseTrajectoryPoint.h>


namespace mm {

void pose_traj_point_to_eigen(const mm_msgs::PoseTrajectoryPoint& msg,
                              Eigen::Vector3d& p, Eigen::Quaterniond& q,
                              Eigen::Vector3d& v, Eigen::Vector3d& w);

} // namespace mm

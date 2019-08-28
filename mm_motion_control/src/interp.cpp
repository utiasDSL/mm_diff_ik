#include "mm_motion_control/interp.h"


namespace mm {

void pose_traj_point_to_eigen(const mm_msgs::PoseTrajectoryPoint& msg,
                              Eigen::Vector3d& p, Eigen::Quaterniond& q,
                              Eigen::Vector3d& v, Eigen::Vector3d& w) {
    p << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    v << msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z;
    q.coeffs() << msg.pose.orientation.x, msg.pose.orientation.y,
                  msg.pose.orientation.z, msg.pose.orientation.w;
    w << msg.velocity.angular.x, msg.velocity.angular.y, msg.velocity.angular.z;
}

} // namespace mm

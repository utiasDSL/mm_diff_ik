#include "mm_msgs/conversions.h"


namespace mm {

void pose_traj_point_to_eigen(const mm_msgs::PoseTrajectoryPoint& msg,
                              Eigen::Vector3d& p, Eigen::Quaterniond& q,
                              Eigen::Vector3d& v, Eigen::Vector3d& w,
                              Eigen::Vector3d& a, Eigen::Vector3d& alpha) {
    // Linear
    p << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    v << msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z;
    a << msg.acceleration.linear.x, msg.acceleration.linear.y, msg.acceleration.linear.z;

    q.coeffs() << msg.pose.orientation.x, msg.pose.orientation.y,
                  msg.pose.orientation.z, msg.pose.orientation.w;
    w << msg.velocity.angular.x, msg.velocity.angular.y, msg.velocity.angular.z;
    alpha << msg.acceleration.angular.x, msg.acceleration.angular.y, msg.acceleration.angular.z;
}


} // namespace mm

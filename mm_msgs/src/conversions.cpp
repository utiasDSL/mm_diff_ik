#include "mm_msgs/conversions.h"


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


void joint_speed_msgs(const Eigen::Matrix<double, 9, 1>& dq,
                      trajectory_msgs::JointTrajectory& traj_arm,
                      geometry_msgs::Twist& twist_base) {

    // Split into base and arm joints. Base is assumed to have 3 joints (x, y,
    // yaw); arm has 6.
    Eigen::Vector3d dq_base = dq.topRows<3>();
    Eigen::Matrix<double, 6, 1> dq_arm = dq.bottomRows<6>();

    // Convert to JointTrajectory message with a single point.
    trajectory_msgs::JointTrajectoryPoint point;
    point.velocities = std::vector<double>(
            dq_arm.data(), dq_arm.data() + dq_arm.size());
    traj_arm.points.push_back(point);

    // Base twist.
    twist_base.linear.x = dq_base(0);
    twist_base.linear.y = dq_base(1);
    twist_base.angular.z = dq_base(2);
}

} // namespace mm

#pragma once

// Utility functions for handling ROS messages.


#include <Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <mm_kinematics/kinematics.h>


namespace mm {


// Populate Pose message from Eigen types.
// Parameters:
//   p:   position
//   q:   orientation
//   msg: Pose message to be populated
inline void pose_msg_from_eigen(const Eigen::Vector3d& p,
                                const Eigen::Quaterniond& q,
                                geometry_msgs::Pose& msg) {
    msg.position.x = p(0);
    msg.position.y = p(1);
    msg.position.z = p(2);

    msg.orientation.w = q.w();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
}


// Convert a vector of joint velocities to ROS messages for base and arm.
// Parameters:
//   dq:         vector of joint velocities
//   traj_arm:   joint trajectory message for the arm
//   twist_base: twist message for the base
inline void joint_speed_msgs(const JointVector& dq,
                      trajectory_msgs::JointTrajectory& traj_arm,
                      geometry_msgs::Twist& twist_base) {

    // Split into base and arm joints. Base is assumed to have 3 joints (x, y,
    // yaw); arm has 6.
    Eigen::Vector3d dq_base = dq.topRows<3>();
    Vector6d dq_arm = dq.bottomRows<6>();

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

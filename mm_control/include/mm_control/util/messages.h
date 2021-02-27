#pragma once

// Utility functions for handling ROS messages.


#include <Eigen/Eigen>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <mm_kinematics/kinematics.h>


namespace mm {


inline Eigen::Vector3d vec3_msg_to_eigen(const geometry_msgs::Vector3& msg) {
   return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Vector3d point_msg_to_eigen(const geometry_msgs::Point& msg) {
   return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Quaterniond quat_msg_to_eigen(const geometry_msgs::Quaternion& msg) {
   return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}


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


inline void twist_msg_from_eigen(const Vector6d& V, geometry_msgs::Twist& msg) {
    msg.linear.x = V(0);
    msg.linear.y = V(1);
    msg.linear.z = V(2);

    msg.angular.x = V(3);
    msg.angular.y = V(4);
    msg.angular.z = V(5);
}

// Populate Eigen types from Pose message.
// Parameters:
//   msg: Pose message
//   p:   position vector to be populated
//   q:   quaternion to be populated
inline void pose_msg_to_eigen(const geometry_msgs::Pose& msg,
                              Eigen::Vector3d& p, Eigen::Quaterniond& q) {
    p << msg.position.x, msg.position.y, msg.position.z;
    q = Eigen::Quaterniond(msg.orientation.w, msg.orientation.x,
                           msg.orientation.y, msg.orientation.z);
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

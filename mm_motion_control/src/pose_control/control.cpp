#include "mm_motion_control/pose_control/control.h"

#include <Eigen/Eigen>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mm_msgs/PoseTrajectory.h>
#include <mm_msgs/CartesianControllerState.h>

#include <mm_kinematics/kinematics.h>
#include <mm_control/control.h>
#include <mm_control/util/messages.h>


namespace mm {


bool CartesianController::init(ros::NodeHandle& nh, const double hz) {
    mm::MMController::init(nh, hz);

    state_pub = nh.advertise<mm_msgs::CartesianControllerState>(
            "/mm_control_state", 1);

    pose_traj_sub = nh.subscribe("/trajectory/poses", 1,
            &CartesianController::pose_traj_cb, this);

    point_traj_sub = nh.subscribe("/trajectory/point", 1,
            &CartesianController::point_traj_cb, this);

    traj_active = false;
}


void CartesianController::pose_traj_cb(const mm_msgs::PoseTrajectory& msg) {
    trajectory.follow(msg);
    traj_active = true;
    ROS_INFO("Trajectory started.");
}


void CartesianController::point_traj_cb(const geometry_msgs::PoseStamped& msg) {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    pose_msg_to_eigen(msg.pose, p, q);
    trajectory.stay_at(p, q);
    traj_active = true;

    ROS_INFO("Maintaining current pose.");
}


void CartesianController::publish_state(const ros::Time& now) {
    double t = now.toSec();

    // Actual pose.
    Eigen::Affine3d w_T_tool;
    Kinematics::calc_w_T_tool(q, w_T_tool);
    Eigen::Vector3d pos_act = w_T_tool.translation();
    Eigen::Matrix3d Re = w_T_tool.rotation();
    Eigen::Quaterniond quat_act(Re);

    // Sample desired trajectory.
    Eigen::Affine3d Td;
    Vector6d Vd;
    trajectory.sample(t, Td, Vd);

    // Desired pose.
    Eigen::Vector3d pos_des = Td.translation();
    Eigen::Matrix3d Rd = Td.rotation();
    Eigen::Quaterniond quat_des(Rd);

    // Error.
    Eigen::Vector3d pos_err = pos_des - pos_act;
    Eigen::Quaterniond quat_err = quat_des * quat_act.inverse();

    mm_msgs::CartesianControllerState msg;
    geometry_msgs::Pose Pa_msg, Pd_msg, P_err_msg;

    pose_msg_from_eigen(pos_act, quat_act, Pa_msg);
    pose_msg_from_eigen(pos_des, quat_des, Pd_msg);
    pose_msg_from_eigen(pos_err, quat_err, P_err_msg);

    geometry_msgs::Twist Vd_msg;
    twist_msg_from_eigen(Vd, Vd_msg);

    // Cartesian space
    msg.actual.pose = Pa_msg;
    msg.desired.pose = Pd_msg;
    msg.error.pose = P_err_msg;

    msg.desired.twist = Vd_msg;

    // Joint space
    msg.joints.position = std::vector<double>(q.data(), q.data() + q.size());
    msg.joints.velocity = std::vector<double>(dq.data(), dq.data() + dq.size());
    msg.command = std::vector<double>(u.data(), u.data() + u.size());

    msg.header.frame_id = "world";
    msg.header.stamp = now;

    state_pub.publish(msg);
}

} // namespace mm

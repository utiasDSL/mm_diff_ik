#include "mm_control/cartesian/control.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>

#include <mm_kinematics/conversion.h>
#include <mm_kinematics/spatial.h>
#include <mm_msgs/CartesianControllerInfo.h>
#include <mm_msgs/CartesianTrajectory.h>

#include <mm_control/cartesian/point.h>
#include <mm_control/cartesian/trajectory.h>
#include <mm_control/control.h>

namespace mm {

bool CartesianController::init(ros::NodeHandle& nh, const double hz) {
  mm::MMController::init(nh, hz);

  info_pub = nh.advertise<mm_msgs::CartesianControllerInfo>(
      "/mm/control/cartesian/info", 1);

  trajectory_sub = nh.subscribe("/mm/control/cartesian/trajectory", 1,
                                &CartesianController::trajectory_cb, this);

  point_sub = nh.subscribe("/mm/control/cartesian/point", 1,
                           &CartesianController::point_cb, this);

  traj_active = false;

  return true;
}

// Control loop.
void CartesianController::loop() {
  ros::Rate rate(hz);
  ROS_INFO("Control loop started, waiting for trajectory...");

  while (ros::ok()) {
    ros::spinOnce();

    ros::Time now = ros::Time::now();

    if (trajectory.initialized() && !trajectory.started()) {
      trajectory.start(now);
      traj_active = true;
      ROS_INFO("Trajectory started.");
    }

    // Do nothing if there is no trajectory active or an unsafe action has
    // been detected.
    if (!traj_active || did_become_unsafe) {
      u = JointVector::Zero();
      publish_joint_speeds(now);
      rate.sleep();
      continue;
    }

    // If the trajectory is over, mark inactive and skip the rest of the
    // loop. Then hits the above condition and proceeds publishing zero.
    if (trajectory.finished(now)) {
      ROS_INFO("Trajectory ended.");
      traj_active = false;
      continue;
    }

    int status = update(now);
    if (status) {
      u = JointVector::Zero();
      did_become_unsafe = true;
    }

    // print but then publish 0
    // ROS_INFO_STREAM("u = " << u);
    // u = JointVector::Zero();

    publish_joint_speeds(now);
    publish_state(now);

    rate.sleep();
  }
}

void CartesianController::trajectory_cb(
    const mm_msgs::CartesianTrajectory& msg) {
  trajectory.init(msg);
}

void CartesianController::point_cb(const geometry_msgs::PoseStamped& msg) {
  Pose pose = pose_msg_to_eigen(msg.pose);
  trajectory.init(pose);
  ROS_INFO("Maintaining current pose.");
}

void CartesianController::publish_state(const ros::Time& now) {
  // Actual pose.
  Transform w_T_tool;
  Kinematics::calc_w_T_tool(q, w_T_tool);

  // Sample desired trajectory.
  CartesianTrajectoryPoint X_des;
  trajectory.sample(now, X_des);

  Pose P_des = X_des.pose;
  Pose P_act(w_T_tool);
  Pose P_err = P_des.error(P_act);

  mm_msgs::CartesianControllerInfo msg;

  // Cartesian space
  pose_msg_from_eigen(P_act, msg.actual.pose);
  pose_msg_from_eigen(P_err, msg.error.pose);
  X_des.message(msg.desired);

  // Joint space
  msg.joints.position = std::vector<double>(q.data(), q.data() + q.size());
  msg.joints.velocity = std::vector<double>(dq.data(), dq.data() + dq.size());
  msg.command = std::vector<double>(u.data(), u.data() + u.size());

  msg.header.frame_id = "world";
  msg.header.stamp = now;

  info_pub.publish(msg);
}

Vector6d calc_cartesian_control_error(const Pose& Pd, const JointVector& q) {
  // Current EE pose.
  Eigen::Affine3d w_T_tool;
  Kinematics::calc_w_T_tool(q, w_T_tool);

  // Orientation error.
  Eigen::Quaterniond Q_act(w_T_tool.rotation());
  Eigen::Quaterniond Q_err = Pd.orientation * Q_act.inverse();

  // Control error: normal position error and the vector part of the
  // quaternion (which is zero when the orientation error is zero).
  Eigen::Vector3d p_err = Pd.position - w_T_tool.translation();
  Eigen::Vector3d rot_err(Q_err.x(), Q_err.y(), Q_err.z());

  Vector6d e;
  e << p_err, rot_err;
  return e;
}

}  // namespace mm

#include "mm_control/cartesian/control.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <mm_msgs/CartesianTrajectory.h>
#include <mm_msgs/CartesianControllerState.h>

#include <mm_control/util/messages.h>
#include <mm_control/cartesian/trajectory.h>
#include <mm_control/control.h>

namespace mm {

bool CartesianController::init(ros::NodeHandle& nh, const double hz) {
  mm::MMController::init(nh, hz);

  state_pub =
      nh.advertise<mm_msgs::CartesianControllerState>("/mm_control_state", 1);

  pose_traj_sub = nh.subscribe("/trajectory/poses", 1,
                               &CartesianController::pose_traj_cb, this);

  point_traj_sub = nh.subscribe("/trajectory/point", 1,
                                &CartesianController::point_traj_cb, this);

  traj_active = false;
}

// Control loop.
void CartesianController::loop() {
  ros::Rate rate(hz);
  ROS_INFO("Control loop started, waiting for trajectory...");

  while (ros::ok()) {
    ros::spinOnce();

    ros::Time now = ros::Time::now();
    double t = now.toSec();

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
    if (trajectory.done(now)) {
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

void CartesianController::pose_traj_cb(
    const mm_msgs::CartesianTrajectory& msg) {
  trajectory.from_message(msg);
  traj_active = true;
  ROS_INFO("Trajectory started.");
}

void CartesianController::point_traj_cb(const geometry_msgs::PoseStamped& msg) {
  Pose pose;
  pose.position << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  pose.orientation.coeffs() << msg.pose.orientation.x, msg.pose.orientation.y,
      msg.pose.orientation.z, msg.pose.orientation.w;
  trajectory.from_point(pose);
  traj_active = true;

  ROS_INFO("Maintaining current pose.");
}

void CartesianController::publish_state(const ros::Time& now) {
  double t = now.toSec();

  // TODO could rewrite a lot of of this much more nicely
  // Actual pose.
  Eigen::Affine3d w_T_tool;
  Kinematics::calc_w_T_tool(q, w_T_tool);
  Pose Pa(w_T_tool);

  // Sample desired trajectory.
  CartesianPosVelAcc Xd;
  trajectory.sample(now, Xd);
  Pose Pd = Xd.pose;

  // Error.
  Eigen::Vector3d pos_err = Pd.position - Pa.position;
  Eigen::Quaterniond quat_err = Pd.orientation * Pa.orientation.inverse();

  mm_msgs::CartesianControllerState msg;
  geometry_msgs::Pose Pa_msg, P_err_msg;

  pose_msg_from_eigen(Pa.position, Pa.orientation, Pa_msg);
  pose_msg_from_eigen(pos_err, quat_err, P_err_msg);

  // Cartesian space
  msg.actual.pose = Pa_msg;
  msg.error.pose = P_err_msg;
  cartesian_state_to_message(Xd, msg.desired);

  // Joint space
  msg.joints.position = std::vector<double>(q.data(), q.data() + q.size());
  msg.joints.velocity = std::vector<double>(dq.data(), dq.data() + dq.size());
  msg.command = std::vector<double>(u.data(), u.data() + u.size());

  msg.header.frame_id = "world";
  msg.header.stamp = now;

  state_pub.publish(msg);
}

}  // namespace mm

#include "mm_control/joint/control.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_msgs/JointControllerInfo.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <mm_control/control.h>
#include <mm_control/joint/trajectory.h>

namespace mm {

bool JointController::init(ros::NodeHandle& nh, const double hz) {
  mm::MMController::init(nh, hz);

  info_pub =
      nh.advertise<mm_msgs::JointControllerInfo>("/mm/control/joint/info", 1);

  trajectory_sub = nh.subscribe("/mm/control/joint/trajectory", 1,
                                &JointController::trajectory_cb, this);

  point_sub = nh.subscribe("/mm/control/joint/point", 1,
                           &JointController::point_cb, this);

  traj_active = false;
}

// Control loop.
void JointController::loop() {
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

    publish_joint_speeds(now);
    publish_state(now);

    rate.sleep();
  }
}

void JointController::trajectory_cb(
    const trajectory_msgs::JointTrajectory& msg) {
  trajectory.init(msg);
}

void JointController::point_cb(
    const trajectory_msgs::JointTrajectoryPoint& msg) {
  trajectory.init(msg);
  ROS_INFO("Received setpoint.");
}

void JointController::publish_state(const ros::Time& now) {
  mm_msgs::JointControllerInfo msg;
  msg.actual.positions = std::vector<double>(q.data(), q.data() + q.size());
  msg.actual.velocities = std::vector<double>(dq.data(), dq.data() + dq.size());

  JointTrajectoryPoint X_des;
  trajectory.sample(now, X_des);
  X_des.message(msg.desired);

  // TODO error

  msg.command = std::vector<double>(u.data(), u.data() + u.size());

  msg.header.frame_id = "world";
  msg.header.stamp = now;

  info_pub.publish(msg);
}

}  // namespace mm

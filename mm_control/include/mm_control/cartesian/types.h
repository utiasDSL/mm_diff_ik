#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_kinematics/conversion.h>
#include <mm_kinematics/spatial.h>
#include <mm_msgs/CartesianTrajectory.h>
#include <mm_msgs/CartesianTrajectoryPoint.h>

namespace mm {

struct CartesianPosVelAcc {
  Pose pose;
  Twist twist;
  Twist acceleration;
};

struct CartesianTrajectoryPoint {
  double time;
  CartesianPosVelAcc state;
};

inline CartesianPosVelAcc cartesian_state_to_eigen(
    const mm_msgs::CartesianState& msg) {
  CartesianPosVelAcc state;
  state.pose = pose_msg_to_eigen(msg.pose);
  state.twist = twist_msg_to_eigen(msg.twist);
  state.acceleration = twist_msg_to_eigen(msg.acceleration);
  return state;
}

inline void cartesian_state_from_eigen(const CartesianPosVelAcc& state,
                                       mm_msgs::CartesianState& msg) {
  pose_msg_from_eigen(state.pose, msg.pose);
  twist_msg_from_eigen(state.twist, msg.twist);
  twist_msg_from_eigen(state.acceleration, msg.acceleration);
}

}  // namespace mm

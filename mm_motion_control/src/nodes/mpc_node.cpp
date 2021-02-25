#include <ros/ros.h>

#include "mm_motion_control/pose_control/mpc.h"
#include "mm_motion_control/pose_control/rate.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "mm_mpc_node");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("MPC controller started.");

  mm::MPController controller;
  controller.init(nh, mm::CONTROL_RATE);
  controller.loop();

  return 0;
}

#include <ros/ros.h>

#include "mm_motion_control/pose_control/diffik.h"
#include "mm_motion_control/pose_control/rate.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mm_diff_ik_control_node");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Differential IK controller started.");

  mm::DiffIKController controller;
  controller.init(nh, mm::CONTROL_RATE);
  controller.loop();

  return 0;
}

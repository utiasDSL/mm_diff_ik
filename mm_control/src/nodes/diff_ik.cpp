#include <ros/ros.h>

#include <mm_control/cartesian/diffik.h>

static const double HZ = 125;

int main(int argc, char **argv) {
  ros::init(argc, argv, "mm_diff_ik_control_node");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Differential IK controller started.");

  mm::DiffIKController controller;
  controller.init(nh, HZ);
  controller.loop();

  return 0;
}

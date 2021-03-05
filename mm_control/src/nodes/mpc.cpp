#include <ros/ros.h>

#include <mm_control/cartesian/mpc.h>

static const double HZ = 125;

int main(int argc, char **argv) {
  ros::init(argc, argv, "mm_mpc_node");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("MPC controller started.");

  mm::MPController controller;
  controller.init(nh, HZ);
  controller.loop();

  return 0;
}

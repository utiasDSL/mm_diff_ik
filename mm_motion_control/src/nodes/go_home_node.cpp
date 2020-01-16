#include <ros/ros.h>

#include "mm_motion_control/joint_control/manager.h"


static const double HZ = 125;


int main(int argc, char **argv) {
  ros::init(argc, argv, "home_trajectory_node");
  ros::NodeHandle nh;

  mm::JointControllerManager manager;
  manager.init(nh);
  manager.loop(HZ);

  return 0;
}

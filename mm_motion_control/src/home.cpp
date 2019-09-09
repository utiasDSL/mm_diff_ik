#include <ros/ros.h>

#include "mm_motion_control/joint.h"


static const double HZ = 125;


int main(int argc, char **argv) {
  ros::init(argc, argv, "home_trajectory_node");
  ros::NodeHandle nh;

  mm::JointControlNode node;
  node.init(nh);
  node.loop(HZ);

  return 0;
}

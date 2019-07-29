#include <ros/ros.h>

#include "mm/mm.h"
#include "mm/control.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "ik_control_node");
  ros::NodeHandle nh;

  ROS_INFO("ik_control_node started");

  mm::IKControlNode node;
  node.init(nh);
  node.loop(125);

  return 0;
}

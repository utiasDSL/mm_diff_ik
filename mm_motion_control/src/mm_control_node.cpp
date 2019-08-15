#include <ros/ros.h>

#include "mm_motion_control/control.h"


static const double HZ = 125;


int main(int argc, char **argv) {
  ros::init(argc, argv, "mm_control_node");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("mm_control_node started at " << HZ << "Hz");

  mm::IKControlNode node;
  node.init(nh);
  node.loop(HZ);

  return 0;
}

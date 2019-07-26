#include <ros/ros.h>

#include "rr/rr.h"
#include "rr/control.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "ik_control_node");
  ros::NodeHandle nh;

  ROS_INFO("ik_control_node started");
  ros::spin();

  return 0;
}

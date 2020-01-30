#include <ros/ros.h>

#include "mm_motion_control/pose_control/manager.h"
#include "mm_motion_control/pose_control/rate.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "mm_pose_control_node");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Pose control node started.");

  mm::IKControllerManager manager;
  manager.init(nh);
  manager.loop(mm::CONTROL_RATE);

  return 0;
}

#include <ros/ros.h>

#include "mm_motion_control/pose_control/manager.h"


static const double HZ = 125;


int main(int argc, char **argv) {
  ros::init(argc, argv, "mm_pose_control_node");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Pose control node started.");

  mm::IKControllerManager manager;
  manager.init(nh);
  manager.loop(HZ);

  return 0;
}

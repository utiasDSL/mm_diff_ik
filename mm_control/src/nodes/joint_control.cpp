#include <ros/ros.h>

#include <mm_kinematics/kinematics.h>

#include <mm_control/joint/pd.h>

static const double HZ = 125;

// Proportional gain matrix.
const static mm::JointMatrix Kp = 0.5 * mm::JointMatrix::Identity();

int main(int argc, char** argv) {
  ros::init(argc, argv, "mm_control_node");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Joint controller started.");

  mm::PDJointController controller;
  controller.init(nh, HZ, Kp);
  controller.loop();

  return 0;
}

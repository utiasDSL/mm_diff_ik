#include <ros/ros.h>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/joint_control/manager.h"


static const double HZ = 125;

// Default home positions -- this can also be set by parameter.
static const mm::JointVector DEFAULT_HOME((mm::JointVector()
            << 0.0, 0.0, 0.0,
               0.0, -0.75*M_PI, -M_PI_2, -0.75*M_PI, -M_PI_2, M_PI_2).finished());


int main(int argc, char **argv) {
  ros::init(argc, argv, "home_trajectory_node");
  ros::NodeHandle nh;

  mm::JointVector q_des = DEFAULT_HOME;

  // If a home position was passed via parameter, use that as the desired
  // joint configuration.
  std::vector<double> joint_home_positions(mm::NUM_JOINTS);
  if (nh.getParam("/joint_home_positions", joint_home_positions)) {
      ROS_INFO_STREAM("Home joint configuration set from parameter.");
      q_des = mm::JointVector(joint_home_positions.data());
  } else {
      ROS_INFO_STREAM("Using default home joint configuration.");
  }

  ROS_INFO("Going home...");

  mm::JointControllerManager manager;
  manager.init(nh, q_des);
  manager.loop(HZ);

  ROS_INFO("Done");

  return 0;
}

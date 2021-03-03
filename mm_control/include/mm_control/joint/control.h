#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <mm_control/control.h>
#include <mm_control/joint/trajectory.h>

namespace mm {

// Abstract base class for joint-space controllers.
class JointController : public MMController {
 public:
  JointController() {}
  ~JointController() {}

  bool init(ros::NodeHandle& nh, const double hz);

  void loop();

 protected:
  /** VARIABLES **/

  ros::Publisher info_pub;

  ros::Subscriber trajectory_sub;
  ros::Subscriber point_sub;

  // Trajectory interpolator.
  JointTrajectory trajectory;

  /** FUNCTIONS **/

  // Receive a trajectory of waypoints to follow.
  void trajectory_cb(const trajectory_msgs::JointTrajectory& msg);

  // Receive a desired set point.
  void point_cb(const trajectory_msgs::JointTrajectoryPoint& msg);

  void publish_state(const ros::Time& now);
};  // class JointController

}  // namespace mm

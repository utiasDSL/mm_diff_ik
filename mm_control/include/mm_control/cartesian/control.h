#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <mm_msgs/CartesianTrajectory.h>

#include <mm_control/cartesian/trajectory.h>
#include <mm_control/control.h>

namespace mm {

// Abstract base class for Cartesian controllers, which track task-space goals.
class CartesianController : public MMController {
 public:
  CartesianController() {}
  ~CartesianController() {}

  bool init(ros::NodeHandle& nh, const double hz);

  void loop();

 protected:
  /** VARIABLES **/

  ros::Publisher state_pub;

  // Subscribe to desired end effector pose trajectories.
  ros::Subscriber pose_traj_sub;
  ros::Subscriber point_traj_sub;

  // Trajectory interpolator.
  CartesianTrajectory trajectory;

  // True if we currently have a trajectory to follow, false otherwise.
  bool traj_active;

  /** FUNCTIONS **/

  // Receive a trajectory of waypoints to follow.
  void pose_traj_cb(const mm_msgs::CartesianTrajectory& msg);

  // Receive a command to keep the EE in place (but the motion control
  // loop is still running so it can respond to e.g. applied forces).
  void point_traj_cb(const geometry_msgs::PoseStamped& msg);

  void publish_state(const ros::Time& now);
};  // class CartesianController

// Calculate the 6D pose error for control.
void calc_cartesian_control_error(const Pose& Pd,
                                  const JointVector& q,
                                  Vector6d& e);

}  // namespace mm

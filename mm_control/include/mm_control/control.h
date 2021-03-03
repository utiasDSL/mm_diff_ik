#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <geometry_msgs/Twist.h>
#include <mm_kinematics/kinematics.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace mm {

// Base controller from which others must be derived.
class MMController {
 public:
  MMController();
  ~MMController();

  // Initialize the node: setup subscribers and publishers, etc.
  bool init(ros::NodeHandle& nh, const double hz);

  // Enter control loop.
  virtual void loop() = 0;

 protected:
  /* VARIABLES */

  // Set to true if an unsafe action is detected so the robot can be
  // stopped.
  bool did_become_unsafe;

  // True if we currently have a trajectory to follow, false otherwise.
  bool traj_active;

  // True if a joint state message has been received, false until then.
  bool joint_state_rec;

  // Controller timestep and rate. dt = 1 / hz
  double dt, hz;

  // Subscriber for current joint state of the robot.
  ros::Subscriber mm_joint_states_sub;

  // Publishers for joint speed commands.
  ros::Publisher ur10_joint_vel_pub;
  ros::Publisher rb_joint_vel_pub;

  JointVector q;
  JointVector dq;
  JointVector u;

  // Time when the last joint state message was received.
  double last_joint_state_time;

  /* FUNCTIONS */

  // Callback to update joint positions and velocities from the robot.
  void mm_joint_states_cb(const sensor_msgs::JointState& msg);

  // Publish joint speed commands to the robot. Should be called from
  // loop().
  void publish_joint_speeds(const ros::Time& now);

  // Generate new control commands. Should be called from loop().
  virtual int update(const ros::Time& now) = 0;

  // Publish state of the controller, for logging and analysis. Should be
  // called from loop().
  virtual void publish_state(const ros::Time& now) = 0;

};  // class MMController

// Convert a vector of joint velocities to ROS messages for base and arm.
// Parameters:
//   u:        vector of joint velocity inputs
//   arm_msg:  joint trajectory message for the arm
//   base_msg: twist message for the base
inline void joint_speed_msgs_from_eigen(
    const JointVector& u,
    trajectory_msgs::JointTrajectory& arm_msg,
    geometry_msgs::Twist& base_msg) {
  // Arm: convert to JointTrajectory message with a single point.
  Vector6d ua = u.bottomRows<NUM_ARM_JOINTS>();
  trajectory_msgs::JointTrajectoryPoint point;
  point.velocities = std::vector<double>(ua.data(), ua.data() + ua.size());
  arm_msg.points.push_back(point);

  // Base twist.
  Eigen::Vector3d ub = u.topRows<NUM_BASE_JOINTS>();
  base_msg.linear.x = ub(0);
  base_msg.linear.y = ub(1);
  base_msg.angular.z = ub(2);
}

}  // namespace mm

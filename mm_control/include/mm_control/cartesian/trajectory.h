#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_kinematics/spatial.h>
#include <mm_math_util/interp.h>
#include <mm_msgs/CartesianTrajectory.h>
#include <mm_msgs/CartesianTrajectoryPoint.h>

namespace mm {

struct CartesianPosVelAcc {
  Pose pose;
  Twist twist;
  Twist acc;
};

struct CartesianTrajectoryPoint {
  double time;
  CartesianPosVelAcc state;
};

class CartesianSplineSegment {
 public:
  // Initialize and interpolate between the provided points.
  CartesianSplineSegment(CartesianTrajectoryPoint& a,
                         CartesianTrajectoryPoint& b);

  // Sample the spline. The given time is relative to the start of the
  // trajectory.
  bool sample(double time, CartesianPosVelAcc& state);

  // time is relative to the start of the trajectory
  double time;
  double duration;

 private:
  // Quintic interpolation for the translational component.
  QuinticInterp<3> pos_interp;

  // Spherical linear interpolation (slerp) for the angular component. Angular
  // velocity and acceleration are assumed to be constant.
  QuaternionInterp quat_interp;
  Eigen::Vector3d angular_vel;
  Eigen::Vector3d angular_acc;
};

class Trajectory {
 public:
  Trajectory() {}
  ~Trajectory() {}

  virtual bool start(const ros::Time& time) = 0;
  virtual bool done(const ros::Time& time) = 0;

 protected:
  double start_time;
  double duration;

  size_t index;
  bool stationary;
};

class CartesianTrajectory : public Trajectory {
 public:
  CartesianTrajectory() {}
  ~CartesianTrajectory() {}

  // Initialize from a list of waypoints.
  bool from_waypoints(std::vector<CartesianTrajectoryPoint> points);

  // Initialize from a ROS message.
  bool from_message(mm_msgs::CartesianTrajectory msg);

  // Initialize from a single point, which means we stay there.
  bool from_point(Pose pose);

  bool start(const ros::Time& time);
  bool sample(const ros::Time& time, CartesianPosVelAcc& state);
  bool done(const ros::Time& time);

 protected:
  // First and last points in the trajectory.
  CartesianPosVelAcc first, last;

  // Spline segments making up the trajectory
  std::vector<CartesianSplineSegment> segments;
};

inline void cartesian_state_to_message(const CartesianPosVelAcc& state,
                                mm_msgs::CartesianState& msg) {
  msg.pose.position.x = state.pose.position(0);
  msg.pose.position.y = state.pose.position(1);
  msg.pose.position.z = state.pose.position(2);

  msg.pose.orientation.x = state.pose.orientation.x();
  msg.pose.orientation.y = state.pose.orientation.y();
  msg.pose.orientation.z = state.pose.orientation.z();
  msg.pose.orientation.w = state.pose.orientation.w();

  msg.twist.linear.x = state.twist.linear(0);
  msg.twist.linear.y = state.twist.linear(1);
  msg.twist.linear.z = state.twist.linear(2);

  msg.twist.angular.x = state.twist.angular(0);
  msg.twist.angular.y = state.twist.angular(1);
  msg.twist.angular.z = state.twist.angular(2);

  msg.acceleration.linear.x = state.acc.linear(0);
  msg.acceleration.linear.y = state.acc.linear(1);
  msg.acceleration.linear.z = state.acc.linear(2);

  msg.acceleration.angular.x = state.acc.angular(0);
  msg.acceleration.angular.y = state.acc.angular(1);
  msg.acceleration.angular.z = state.acc.angular(2);
}

}  // namespace mm

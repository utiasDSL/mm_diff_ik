#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/interp.h>
#include <mm_msgs/PoseTrajectory.h>
#include <mm_msgs/PoseTrajectoryPoint.h>
#include <mm_msgs/conversions.h>

namespace mm {

struct CartesianPosVelAcc {
  Eigen::Affine3d pose;
  Vector6d twist;
  Vector6d acc;
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

 private:
  // time is relative to the start of the trajectory
  double time;
  double duration;

  // Quintic interpolation for the translational component.
  QuinticInterp<3> pos_interp;

  // Spherical linear interpolation (slerp) for the angular component.
  QuaternionInterp quat_interp;
};

class Trajectory {
 public:
  Trajectory();
  ~Trajectory();

  virtual bool start(const ros::Time& time) = 0;
  virtual bool done(const ros::Time& time) = 0;

 private:
  double start_time;
  double duration;

  size_t index;
  bool stationary;
};

class CartesianTrajectory : public Trajectory {
 public:
  CartesianTrajectory();
  ~CartesianTrajectory();

  // Initialize from a list of waypoints.
  bool from_waypoints(std::vector<CartesianTrajectoryPoint> points);

  // Initialize from a ROS message.
  bool from_message(mm_msgs::CartesianTrajectory msg);

  // Initialize from a single point, which means we stay there.
  bool from_point(Eigen::Affine3d pose);

  bool start(const ros::Time& time);
  bool sample(const ros::Time& time, CartesianPosVelAcc& state);
  bool done(const ros::Time& time);

 private:
  // First and last points in the trajectory.
  CartesianPosVelAcc first, last;

  // Spline segments making up the trajectory
  std::vector<CartesianSplineSegment> segments;
};

}  // namespace mm

#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_control/cartesian/spline.h>
#include <mm_control/cartesian/types.h>
#include <mm_control/trajectory.h>
#include <mm_kinematics/spatial.h>
#include <mm_msgs/CartesianTrajectory.h>

namespace mm {

// Trajectory in Cartesian (task) space.
class CartesianTrajectory : public Trajectory {
 public:
  CartesianTrajectory() {}
  ~CartesianTrajectory() {}

  // Initialize from a list of waypoints.
  bool init(const std::vector<CartesianTrajectoryPoint>& points);

  // Initialize from a ROS message.
  bool init(const mm_msgs::CartesianTrajectory& msg);

  // Initialize from a single setpoint.
  bool init(const Pose& pose);

  // Sample the trajectory at the given time `now`. Returns false if the trajectory
  // is sampled in an invalid state, in which case the `state` variable should
  // not be trusted. Trajectories are assumed to be sampled at monotonically
  // increasing times.
  bool sample(const ros::Time& now, CartesianTrajectoryState& state);

  // Sample the trajectory at `times` in the after the given time `now`. The
  // `times` are expected to be monotonically increasing.
  bool sample(const ros::Time& now,
              const std::vector<ros::Time>& times,
              std::vector<CartesianTrajectoryState>& states);

 protected:
  // First and last points in the trajectory.
  CartesianTrajectoryState first, last;

  // Spline segments making up the trajectory
  std::vector<CartesianSplineSegment> segments;
};

}  // namespace mm

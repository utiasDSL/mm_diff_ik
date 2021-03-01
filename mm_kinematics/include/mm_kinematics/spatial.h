#pragma once

#include <Eigen/Eigen>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

// We make the distinction between poses and transforms, even though they both
// encode a position and orientation in 3D space. Transforms are meant
// application to points to change the frame in which they are expressed. Poses
// are meant to represent the position and orientation of a particular point in
// a particular frame.

// For transforms we just use Eigen's built-in type.
typedef Eigen::Affine3d Transform;

// For poses, it is more convenient to hold a position vector and orientation
// quaternion, to mirror the ROS message format.
struct Pose {
  // Default constructor: zero pose.
  Pose() : position(0, 0, 0), orientation(1, 0, 0, 0) {}

  // Initialize directly from position and quaternion.
  Pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q)
      : position(p), orientation(q) {}

  // Initialize from a transform.
  Pose(const Transform& T) {
    position = T.translation();
    orientation = Eigen::Quaterniond(T.rotation());
  }

  // Calculate error between this and another pose.
  Pose error(const Pose& other) {
    return Pose(position - other.position,
                orientation * other.orientation.inverse());
  }

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

// Twist represents a 6-DOF velocity in 3D space.
struct Twist {
  Twist() : linear(0, 0, 0), angular(0, 0, 0) {}

  Twist(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular)
      : linear(linear), angular(angular) {}

  // Get the twist as a single 6-dimensional vector.
  Vector6d vector() {
    Vector6d V;
    V << linear, angular;
    return V;
  }

  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

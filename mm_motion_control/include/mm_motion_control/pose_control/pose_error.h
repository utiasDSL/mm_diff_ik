#pragma once

#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/linalg.h>


namespace mm {

// Calculate pose error.
// Parameters:
//   Td: desired pose
//   q:  current joint positions
//   e:  populated with orientation error vector
void calc_pose_error(const Eigen::Affine3d& Td, const JointVector& q, Vector6d& e);


// Calculate rotation error.
// Parameters:
//   Rd: desired rotation matrix
//   Re: current rotation matrix
//   e:  populated with rotation error
void calc_rotation_error(const Eigen::Matrix3d& Rd, const Eigen::Matrix3d& Re,
                         Eigen::Vector3d& e);

} // namespace mm

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


// Calculate pose error Jacobian (note this is different from the manipulator
// Jacobian).
// Parameters:
//   Td: desired pose
//   q:  current joint positions
//   J:  populated with pose error Jacobian
void calc_pose_error_jacobian(const Eigen::Affine3d& Td, const JointVector& q,
                         JacobianMatrix& J);


// Calculate rotation error.
// Parameters:
//   Rd: desired rotation matrix
//   Re: current rotation matrix
//   e:  populated with rotation error
void calc_rotation_error(const Eigen::Matrix3d& Rd, const Eigen::Matrix3d& Re,
                    Eigen::Vector3d& e);


// Calculate Jacobians w.r.t. each column of the forward kinematics rotation
// matrix, R(q) = [n, s, a].
// Parameters:
//   q: current joint positions
//   Jn: populated with Jacobian of first column (n)
//   Js: populated with Jacobian of second column (s)
//   Ja: populated with Jacobian of third column (a)
void calc_rotation_error_jacobians(const JointVector& q,
                              Matrix3x9& Jn, Matrix3x9& Js, Matrix3x9& Ja);

} // namespace mm

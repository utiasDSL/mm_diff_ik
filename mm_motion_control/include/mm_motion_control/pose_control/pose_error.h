#pragma once

#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/linalg.h>


namespace mm {

// Calculate Jacobians w.r.t. each column of the forward kinematics rotation
// matrix, R(q) = [n, s, a].
// Parameters:
//   q: current joint positions
//   Jn: populated with Jacobian of first column (n)
//   Js: populated with Jacobian of second column (s)
//   Ja: populated with Jacobian of third column (a)
void rotation_error_jacobians(const JointVector& q,
                              Matrix3x9& Jn, Matrix3x9& Js, Matrix3x9& Ja);


// Calculate rotation error.
// Parameters:
//   d: desired orientation
//   q: current joint positions
//   e: populated with orientation error vector
void rotation_error(const Eigen::Quaterniond& d, const JointVector& q,
                    Eigen::Vector3d& e);


// Linearize the rotation error around the joint configuration q.
// Parameters:
//   d:  desired orientation
//   q:  current joint positions
//   dt: time step
//   e:  populated with orientation error calculated at the linearization point q
//   J:  populated with Jacobian (first derivative) of orientation error
//       evaluated at the linearization point
void linearize_rotation_error(const Eigen::Quaterniond& d, const JointVector& q,
                              double dt, Eigen::Vector3d& e, Matrix3x9& J);


// Linearize the position error around the joint configuration q.
// Parameters:
//   d:  desired position
//   q:  current joint positions
//   dt: time step
//   e:  populated with position error calculated at the linearization point q
//   J:  populated with Jacobian (first derivative) of position error evaluated
//       at the linearization point
void linearize_position_error(const Eigen::Vector3d& d, const JointVector& q,
                              double dt, Eigen::Vector3d& e, Matrix3x9& J);

} // namespace mm

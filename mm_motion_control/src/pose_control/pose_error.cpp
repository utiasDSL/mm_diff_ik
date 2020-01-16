#include "mm_motion_control/pose_control/pose_error.h"

#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/linalg.h>


namespace mm {

void rotation_error(const Eigen::Quaterniond& d, const JointVector& q,
                    Eigen::Vector3d& e) {
    // Desired EE rotation.
    Eigen::Matrix3d Rd = d.toRotationMatrix();

    // Current EE rotation (extract from current pose).
    Eigen::Affine3d w_T_e;
    Kinematics::calc_w_T_e(q, w_T_e);
    Eigen::Matrix3d Re = w_T_e.rotation();

    Eigen::Vector3d nd = Rd.col(0);
    Eigen::Vector3d sd = Rd.col(1);
    Eigen::Vector3d ad = Rd.col(2);

    Eigen::Vector3d ne = Re.col(0);
    Eigen::Vector3d se = Re.col(1);
    Eigen::Vector3d ae = Re.col(2);

    e = 0.5 * (ne.cross(nd) + se.cross(sd) + ae.cross(ad));
}


void linearize_rotation_error(const Eigen::Quaterniond& d, const JointVector& q,
                              double dt, Eigen::Vector3d& e, Matrix3x9& J) {
    // Desired EE rotation.
    Eigen::Matrix3d Rd = d.toRotationMatrix();

    // Current EE rotation (extract from current pose).
    Eigen::Affine3d w_T_e;
    Kinematics::calc_w_T_e(q, w_T_e);
    Eigen::Matrix3d Re = w_T_e.rotation();

    // Skew matrices to take the cross product.
    Eigen::Matrix3d Nd = skew(Rd.col(0));
    Eigen::Matrix3d Sd = skew(Rd.col(1));
    Eigen::Matrix3d Ad = skew(Rd.col(2));

    Eigen::Vector3d ne = Re.col(0);
    Eigen::Vector3d se = Re.col(1);
    Eigen::Vector3d ae = Re.col(2);

    // e = r * sin(phi), where (r, phi) are the (axis, angle) representing the
    // orientation error. See Siciliano et al., 2010, pg. 139 for details.
    // Negative sign is the result of reversing the order of the cross products.
    e = -0.5 * (Nd * ne + Sd * se + Ad * ae);

    // Construct Jacobian of e for first-order linearization.
    Matrix3x9 Jn, Js, Ja;
    rotation_error_jacobians(q, Jn, Js, Ja);
    J = -0.5 * dt * (Nd * Jn + Sd * Js + Ad * Ja);
}


void linearize_position_error(const Eigen::Vector3d& d, const JointVector& q,
                              double dt, Eigen::Vector3d& e, Matrix3x9& J) {

    // Calculate actual pose using forward kinematics.
    Eigen::Affine3d w_T_e;
    Kinematics::calc_w_T_e(q, w_T_e);
    Eigen::Vector3d p = w_T_e.translation();
    e = d - p;

    // Jacobian for position error is just the top three rows of the usual
    // geometric Jacobian.
    JacobianMatrix J_geo;
    Kinematics::jacobian(q, J_geo);
    J = -dt * J_geo.topRows<3>();
}

} // namespace mm

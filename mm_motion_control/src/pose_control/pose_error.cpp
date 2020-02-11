#include "mm_motion_control/pose_control/pose_error.h"

#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/linalg.h>


namespace mm {

/*** PRIVATE ***/

void position_error_jacobian(const JointVector& q, Matrix3x9& Jp) {
    JacobianMatrix J_geo; // geometric Jacobian
    Kinematics::jacobian(q, J_geo);

    // Note negative sign since error is pd - pe.
    Jp = -J_geo.topRows<3>();
}


/*** PUBLIC ***/

void rotation_error(const Eigen::Matrix3d& Rd, const Eigen::Matrix3d& Re,
                    Eigen::Vector3d& e) {
    Eigen::Vector3d nd = Rd.col(0);
    Eigen::Vector3d sd = Rd.col(1);
    Eigen::Vector3d ad = Rd.col(2);

    Eigen::Vector3d ne = Re.col(0);
    Eigen::Vector3d se = Re.col(1);
    Eigen::Vector3d ae = Re.col(2);

    // e = r * sin(phi), where (r, phi) are the (axis, angle) representing the
    // orientation error. See Siciliano et al., 2010, pg. 139 for details.
    e = 0.5 * (ne.cross(nd) + se.cross(sd) + ae.cross(ad));
}


void rotation_error_jacobian(const Eigen::Matrix3d& Rd, const JointVector& q,
                             Matrix3x9& Je) {
    // Skew matrices to take the cross product.
    Eigen::Matrix3d Nd = skew(Rd.col(0));
    Eigen::Matrix3d Sd = skew(Rd.col(1));
    Eigen::Matrix3d Ad = skew(Rd.col(2));

    Matrix3x9 Jn, Js, Ja;
    rotation_error_jacobians(q, Jn, Js, Ja);

    Je = -0.5 * (Nd * Jn + Sd * Js + Ad * Ja);
}


void pose_error(const Eigen::Affine3d& Td, const JointVector& q, Vector6d& e) {
    Eigen::Matrix3d Rd = Td.rotation();
    Eigen::Vector3d pd = Td.translation();

    // Current EE pose.
    Eigen::Affine3d w_T_tool;
    Kinematics::calc_w_T_tool(q, w_T_tool);
    Eigen::Matrix3d Re = w_T_tool.rotation();
    Eigen::Vector3d pe = w_T_tool.translation();

    // Calculate errors.
    Eigen::Vector3d pos_err, rot_err;
    pos_err = pd - pe;
    rotation_error(Rd, Re, rot_err);

    e << pos_err, rot_err;
}


void pose_error_jacobian(const Eigen::Affine3d& Td, const JointVector& q,
                         JacobianMatrix& J) {
    Matrix3x9 Jp, Je;
    rotation_error_jacobian(Td.rotation(), q, Je);
    position_error_jacobian(q, Jp);
    J << Jp, Je;
}


} // namespace mm

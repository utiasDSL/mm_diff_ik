#include "mm_motion_control/pose_control/pose_error.h"

#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/linalg.h>


namespace mm {

void calc_rotation_error(const Eigen::Matrix3d& Rd, const Eigen::Matrix3d& Re,
                    Eigen::Vector3d& e) {
    Eigen::Matrix3d R_err = Rd * Re.transpose();
    Eigen::Quaterniond quat(R_err);
    e << quat.x(), quat.y(), quat.z();
}

void calc_pose_error(const Eigen::Affine3d& Td, const JointVector& q, Vector6d& e) {
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
    calc_rotation_error(Rd, Re, rot_err);

    e << pos_err, rot_err;
}


} // namespace mm

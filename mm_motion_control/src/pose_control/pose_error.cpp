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

void calc_pose_error(const Pose& Pd, const JointVector& q, Vector6d& e) {
    // Current EE pose.
    Eigen::Affine3d w_T_tool;
    Kinematics::calc_w_T_tool(q, w_T_tool);

    // Orientation error.
    Eigen::Quaterniond Q_act(w_T_tool.rotation());
    Eigen::Quaterniond Q_err = Pd.orientation * Q_act.inverse();

    // Control error: normal position error and the vector part of the
    // quaternion (which is zero when the orientation error is zero).
    Eigen::Vector3d p_err = Pd.position - w_T_tool.translation();
    Eigen::Vector3d rot_err(Q_err.x(), Q_err.y(), Q_err.z());

    e << p_err, rot_err;
}


} // namespace mm

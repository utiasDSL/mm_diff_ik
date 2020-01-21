#include "mm_motion_control/pose_control/controller.h"

#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/pose_control/trajectory.h"
#include "mm_motion_control/pose_control/obstacle.h"


namespace mm {


bool IKController::init(double t0) {
    optimizer.init();
    set_time(t0);
    return true;
}


void IKController::set_time(double t) {
    time_prev = t;
}


int IKController::update(double t, PoseTrajectory& trajectory,
                         const JointVector& q_act, const JointVector& dq_act,
                         const std::vector<ObstacleModel> obstacles,
                         JointVector& dq_cmd) {
    // Update time.
    double dt = t - time_prev;
    time_prev = t;

    ROS_INFO_STREAM("dt = " << dt);

    // Optimize to solve IK problem.
    return optimizer.solve(t, trajectory, q_act, dq_act, dt, dq_cmd);
}

} // namespace mm

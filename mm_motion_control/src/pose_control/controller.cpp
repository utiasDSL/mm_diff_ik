#include "mm_motion_control/pose_control/controller.h"

#include <Eigen/Eigen>
#include <ros/ros.h>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/pose_control/trajectory.h"
#include "mm_motion_control/pose_control/obstacle.h"
#include "mm_motion_control/pose_control/rate.h"


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
                         const JointVector& q, const JointVector& dq,
                         double fd, const Eigen::Vector3d& f,
                         const Eigen::Vector3d& pc,
                         const std::vector<ObstacleModel> obstacles,
                         JointVector& u) {
    // Update time.
    double dt = t - time_prev;
    time_prev = t;

    // May improve stability to set the timestep as a fixed value (assuming it
    // is accurate).
    dt = CONTROL_TIMESTEP;

    // Optimize to solve IK problem.
    // double t1 = ros::Time::now().toSec();
    int status = optimizer.solve(t, trajectory, q, dq, fd, f, pc, obstacles, dt, u);  // IK
    // int status = optimizer.solve(t, trajectory, q, dq, obstacles, dt, u);  // MPC
    // double dt2 = ros::Time::now().toSec() - t1;

    // ROS_INFO_STREAM("loop dt = " << dt << ", opt dt = " << dt2);

    return status;
}

} // namespace mm

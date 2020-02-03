#pragma once

#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/pose_control/optimizer.h"
#include "mm_motion_control/pose_control/mpc_optimizer.h"
#include "mm_motion_control/pose_control/trajectory.h"
#include "mm_motion_control/pose_control/obstacle.h"


namespace mm {


// The IKController wraps the IKOptimizer to perform additional preprocessing
// and publishing.
class IKController {
    public:
        IKController() : optimizer() {}

        // Initialize the controller
        // Parameters:
        //   t: current time (sec)
        bool init(double t);

        // Run one iteration of the control loop.
        // Parameters:
        //   t:          current time (sec)
        //   trajectory: desired EE pose trajectory
        //   q_act:      current joint angles
        //   dq_act:     current joint velocities
        //   obstacles:  list of currently perceived obstacles
        //   dq_cmd:     populated with joint velocity commands to send
        //
        // Returns:
        //   0 if the optimization problem was solved sucessfully, otherwise a
        //   non-zero status code.
        int update(double t, PoseTrajectory& trajectory,
                   const JointVector& q_act, const JointVector& dq_act,
                   const Eigen::Vector3d& force,
                   const std::vector<ObstacleModel> obstacles,
                   JointVector& dq_cmd);

        // Reset the stored previous time to time t.
        void set_time(double t);

    private:
        // Time from previous control loop iteration.
        double time_prev;

        // Optimizer to solve for joint velocity commands to send to the
        // robot.
        IKOptimizer optimizer;

}; // class IKController

} // namespace mm

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
// TODO I think we can get rid of separation between the two now: the
// controller basically does nothing
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
        //   q:          current joint angles
        //   dq:         current joint velocities
        //   obstacles:  list of currently perceived obstacles
        //   u:          populated with joint velocity commands to send
        //
        // Returns:
        //   0 if the optimization problem was solved sucessfully, otherwise a
        //   non-zero status code.
        int update(double t, PoseTrajectory& trajectory,
                   const JointVector& q, const JointVector& dq,
                   double fd, const Eigen::Vector3d& force,
                   const Eigen::Vector3d& torque,
                   const Eigen::Vector3d& pc,
                   const std::vector<ObstacleModel> obstacles,
                   JointVector& u);

        // Reset the stored previous time to time t.
        void set_time(double t);

        // Get detailed state of the optimizer.
        void get_optimizer_state(IKOptimizerState& state);

    private:
        // Time from previous control loop iteration.
        double time_prev;

        // Optimizer to solve for joint velocity commands to send to the
        // robot.
        // MPCOptimizer optimizer;
        IKOptimizer optimizer;

}; // class IKController

} // namespace mm

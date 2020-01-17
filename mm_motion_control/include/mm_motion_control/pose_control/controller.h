#pragma once

#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>
#include <mm_msgs/PoseTrajectory.h>
#include <mm_msgs/PoseControlState.h>
#include <mm_msgs/Obstacles.h>

#include "mm_motion_control/pose_control/optimizer.h"
#include "mm_motion_control/pose_control/trajectory.h"
#include "mm_motion_control/pose_control/obstacle.h"


namespace mm {


// The IKController wraps the IKOptimizer to perform additional preprocessing
// and publishing.
class IKController {
    public:
        IKController() : optimizer() {}

        bool init();

        // Run one iteration of the control loop.
        // Parameters:
        //   pos_des:   desired position
        //   quat_des:  desired orientation
        //   q_act:     current joint angles
        //   dq_act:    current joint velocities
        //   obstacles: list of currently perceived obstacles
        //   dq_cmd:    populated with joint velocity commands to send
        //
        // Returns:
        //   0 if the optimization problem was solved sucessfully, otherwise a
        //   non-zero status code.
        int update(const Eigen::Vector3d& pos_des,
                   const Eigen::Quaterniond& quat_des,
                   const JointVector& q_act,
                   const JointVector& dq_act,
                   const std::vector<ObstacleModel> obstacles,
                   JointVector& dq_cmd);

        // Reset the stored previous time to current time.
        void tick();

    private:
        // Time from previous control loop iteration.
        double time_prev;

        // Proportional gains on the end effector pose error (in task space),
        // for linear and rotational error, respectively.
        Eigen::Matrix3d Kv;
        Eigen::Matrix3d Kw;

        // Optimizer to solve for joint velocity commands to send to the
        // robot.
        IKOptimizer optimizer;

}; // class IKController

} // namespace mm

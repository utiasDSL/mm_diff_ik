#pragma once

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <mm_msgs/OptimizationState.h>
#include <mm_kinematics/kinematics.h>
#include <mm_math_util/differentiation.h>

#include "mm_motion_control/pose_control/obstacle.h"
#include "mm_motion_control/pose_control/pose_error.h"


namespace mm {

// Distances for the velocity damper constraints.
static const double M_PI_6 = M_PI / 6.0;
static const JointVector INFLUENCE_DIST((JointVector() <<
    0.5, 0.5, M_PI_6,                               /* base */
    M_PI_4, M_PI_4, M_PI_4, M_PI_4, M_PI_4, M_PI_4  /* arm */
).finished());

static const JointVector SAFETY_DIST = 0.25 * INFLUENCE_DIST;

// Make the joint true if a position limit should be enforced, false otherwise.
static const bool POSITIION_LIMITED[] = {
    false, false, false,               /* base */
    true, true, true, true, true, true /* arm  */
};

// For numerical differentiation
// NOTE pushing this down to even 1e-5 makes the numerical Hessian fail
static const double STEP_SIZE = 1e-3;


class IKOptimizer {
    public:
        IKOptimizer() {};

        bool init(ros::NodeHandle& nh);

        // Create and solve the QP.
        //
        // q:      Current joint angles.
        // dq:     Current joint velocities.
        // d:      d = pd - f(q)
        // vd:     Desired task space velocity of end effector.
        // dt:     Control timestep.
        // dq_opt: Optimal values of joint velocities.
        //
        // Return value is 0 if the problem was solved successfully. Otherwise,
        // status code indicates a failure in the optmization problem.
        int solve(const Eigen::Vector3d& pos_des,
                  const Eigen::Quaterniond& quat_des, const JointVector& q,
                  const JointVector& dq, const Vector6d& vd,
                  const std::vector<ObstacleModel>& obstacles, double dt,
                  JointVector& dq_opt);

        // 1st-order linearization of manipulability index around joint values
        // q, returning the gradient dm.
        void linearize_manipulability1(const JointVector& q, JointVector& dm,
                                       double h);

        // 2nd-order linearization of manipulability index around joint values
        // q, returning the gradient dm, and Hessian Hm.
        void linearize_manipulability2(const JointVector& q, JointVector& dm,
                                       JointMatrix& Hm, double h);

        // Compute velocity constraints, including damping as the position
        // limits are approached.
        void velocity_damper_limits(const JointVector& q, JointVector& dq_lb,
                                    JointVector& dq_ub);

        // Calculate obstacle avoidance objective.
        int obstacle_limits(const Eigen::Vector2d& pb,
                            const std::vector<ObstacleModel>& obstacles,
                            Eigen::MatrixXd& A, Eigen::VectorXd& b);

        // Filter out obstacles that are not close enough to influence the
        // optimization problem.
        void filter_obstacles(const Eigen::Vector2d& pb,
                              const std::vector<ObstacleModel>& obstacles,
                              std::vector<ObstacleModel>& close_obstacles);

    private:
        typedef realtime_tools::RealtimePublisher<mm_msgs::OptimizationState> StatePublisher;
        typedef std::unique_ptr<StatePublisher> StatePublisherPtr;

        StatePublisherPtr state_pub;

        void publish_state(const JointVector& dq_opt, double vel_obj,
                           double mi_obj);
}; // class IKOptimizer

} // namespace mm

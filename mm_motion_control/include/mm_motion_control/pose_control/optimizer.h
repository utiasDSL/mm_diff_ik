#pragma once

#include <Eigen/Eigen>
#include <qpOASES/qpOASES.hpp>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/pose_control/obstacle.h"
#include "mm_motion_control/pose_control/trajectory.h"


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


// State of the Optimizer
struct IKOptimizerState {
    JointVector dq_opt;  // Optimal joint velocities.
    double obj_val;  // Objective value.

    // Detailed status code returned by optimization problem. 0 indicates
    // success (see qpOASES user manual).
    qpOASES::returnValue code;

    // Simple status code indicating status of optimization problem:
    //  0: QP was solved,
    //  1: QP could not be solved within the given number of iterations,
    // -1: QP could not be solved due to an internal error,
    // -2: QP is infeasible and thus could not be solved,
    // -3: QP is unbounded and thus could not be solved.
    int status;
};


class IKOptimizer {
    public:
        IKOptimizer() : state() {};

        bool init();

        // Create and solve the QP.
        // Parameters:
        //   t:          Current time.
        //   trajectory: Desired EE pose trajectory.
        //   q:          Current joint angles.
        //   dq:         Current joint velocities.
        //   dt:         Control timestep.
        //   dq_opt:     Optimal values of joint velocities.
        //
        // Returns:
        //   0 if the optimization problem was solved successfully. Otherwise,
        //   status code indicates a failure in the optimization problem.
        int solve(double t, PoseTrajectory& trajectory,
                  const JointVector& q, const JointVector& dq,
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

        // Returns the current state of the optimizer.
        void get_state(IKOptimizerState& state);

    private:
        IKOptimizerState state;

        // Construct and solve the QP given our problem-specific matrices.
        // Note that the underlying data for input arguments is not copied.
        int solve_qp(JointMatrix& H, JointVector& g, Eigen::MatrixXd& A,
                     JointVector& lb, JointVector& ub, Eigen::VectorXd& ubA,
                     JointVector& dq_opt);

        // Build objective matrices H and g, where the quadratic to minimize is
        // x'Hx + g'x.
        void build_objective(const Eigen::Affine3d& Td, const Vector6d& twistd,
                             const JointVector& q, const JointVector& dq,
                             double dt, JointMatrix& H, JointVector& g);
}; // class IKOptimizer

} // namespace mm

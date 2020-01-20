#pragma once

#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/pose_control/trajectory.h"


namespace mm {

// TODO Later we will likely specify a time horizon and an interval and
// calculate based on that.
static const int LOOKAHEAD_STEP_TIME = 0.01;
static const int NUM_HORIZON = 10; // steps to look ahead
static const int NUM_ITER = 10; // number of relinearizations in SQP
static const int NUM_WSR = 10; // max number of working set recalculations

static const int NUM_OPT = NUM_JOINTS * NUM_HORIZON;


class MPCOptimizer {
    public:
        MPCOptimizer() : {};

        bool init();

        // Create and solve the QP.
        // Parameters:
        //   traj:   Desired EE pose trajectory.
        //   q0:     Current joint angles.
        //   dq0:    Current joint velocities.
        //   dt:     Control timestep.
        //   dq_opt: Optimal values of joint velocities.
        //
        // Returns:
        //   0 if the optimization problem was solved successfully. Otherwise,
        //   status code indicates a failure in the optimization problem.
        int solve(PoseTrajectory& traj, const JointVector& q0,
                  const JointVector& dq0, double dt, JointVector& dq_opt) {

    private:
        typedef Eigen::Matrix<double, NUM_OPT, 1> OptVector;
        typedef Eigen::Matrix<double, NUM_OPT, NUM_OPT> OptWeightMatrix;

        typedef Eigen::Matrix<double, 6 * NUM_HORIZON, 1> OptErrorVector;
        typedef Eigen::Matrix<double, 6 * NUM_HORIZON, 6 * NUM_HORIZON> OptErrorWeightMatrix;
        typedef Eigen::Matrix<double, 6 * NUM_HORIZON, NUM_JOINTS * NUM_HORIZON> OptLiftedJacobian;

        // Construct and solve the QP given our problem-specific matrices.
        // Note that the underlying data for input arguments is not copied.
        int solve_sqp(qpOASES::SQProblem& sqp, OptWeightMatrix& H,
                      OptVector& g, OptVector& lb, OptVector& ub,
                      bool init, OptVector& step) {

}; // class MPCOptimizer

} // namespace mm

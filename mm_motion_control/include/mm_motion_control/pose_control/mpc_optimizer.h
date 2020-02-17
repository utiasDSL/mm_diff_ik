#pragma once

#include <Eigen/Eigen>
#include <qpOASES/qpOASES.hpp>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/pose_control/obstacle.h"
#include "mm_motion_control/pose_control/trajectory.h"


namespace mm {

static const double LOOKAHEAD_TIMESTEP = 0.08;

static const int NUM_HORIZON = 5; // steps to look ahead
static const int NUM_ITER = 2; // number of relinearizations in SQP
static const int NUM_WSR = 200; // max number of working set recalculations

static const int NUM_OPT = NUM_JOINTS * NUM_HORIZON;


class MPCOptimizer {
    public:
        MPCOptimizer() : sqp(NUM_OPT, 0) {};

        bool init();

        // Create and solve the QP.
        // Parameters:
        //   t:          Current time.
        //   trajectory: Desired EE pose trajectory.
        //   q0:         Current joint angles.
        //   dq0:        Current joint velocities.
        //   dt:         Control timestep.
        //   dq_opt:     Optimal values of joint velocities.
        //
        // Returns:
        //   0 if the optimization problem was solved successfully. Otherwise,
        //   status code indicates a failure in the optimization problem.
        int solve(double t0, PoseTrajectory& trajectory, const JointVector& q0,
                  const JointVector& dq0,
                  const std::vector<ObstacleModel>& obstacles, double dt,
                  JointVector& dq_opt);

    private:
        typedef Eigen::Matrix<double, NUM_OPT, 1> OptVector;
        typedef Eigen::Matrix<double, NUM_OPT, NUM_OPT> OptWeightMatrix;

        typedef Eigen::Matrix<double, 6 * NUM_HORIZON, 1> OptErrorVector;
        typedef Eigen::Matrix<double, 6 * NUM_HORIZON, 6 * NUM_HORIZON> OptErrorWeightMatrix;
        typedef Eigen::Matrix<double, 6 * NUM_HORIZON, NUM_JOINTS * NUM_HORIZON> OptLiftedJacobian;

        // SQP to solve.
        qpOASES::SQProblem sqp;

        // Construct and solve the QP given our problem-specific matrices.
        // Note that the underlying data for input arguments is not copied.
        int solve_sqp(OptWeightMatrix& H, OptVector& g, OptVector& lb,
                      OptVector& ub, OptVector& step);

}; // class MPCOptimizer

} // namespace mm

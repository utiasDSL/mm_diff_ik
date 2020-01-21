#include "mm_motion_control/pose_control/mpc_optimizer.h"

#include <Eigen/Eigen>
#include <qpOASES/qpOASES.hpp>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/differentiation.h>

#include "mm_motion_control/pose_control/pose_error.h"


namespace mm {



bool MPCOptimizer::init() {
    sqp.setPrintLevel(qpOASES::PL_NONE);
    return true;
}


int MPCOptimizer::solve_sqp(OptWeightMatrix& H, OptVector& g, OptVector& lb,
                            OptVector& ub, OptVector& step) {
    // Convert to row-major order. Only meaningful for matrices, not
    // vectors.
    Eigen::Matrix<qpOASES::real_t,
                  NUM_OPT, NUM_OPT,
                  Eigen::RowMajor> H_rowmajor = H;

    // Convert eigen data to raw arrays for qpOASES.
    qpOASES::real_t *H_data = H_rowmajor.data();
    qpOASES::real_t *g_data = g.data();

    qpOASES::real_t *lb_data = lb.data();
    qpOASES::real_t *ub_data = ub.data();

    qpOASES::int_t nWSR = NUM_WSR;

    // Solve the QP.
    qpOASES::returnValue ret;
    if (sqp.isInitialised()) {
        ret = sqp.hotstart(H_data, g_data, NULL, lb_data, ub_data, NULL, NULL, nWSR);
    } else {
        ret = sqp.init(H_data, g_data, NULL, lb_data, ub_data, NULL, NULL, nWSR);
    }

    qpOASES::real_t step_raw[NUM_OPT];
    sqp.getPrimalSolution(step_raw);
    step = Eigen::Map<OptVector>(step_raw); // map back to eigen

    return qpOASES::getSimpleStatus(ret);
}


int MPCOptimizer::solve(double t0, PoseTrajectory& trajectory,
                        const JointVector& q0, const JointVector& dq0,
                        double dt, JointVector& dq_opt) {
    // Error weight matrix.
    Matrix6d Q = Matrix6d::Identity();
    Q.topLeftCorner<3, 3>() = 100 * Eigen::Matrix3d::Identity();
    Q.bottomRightCorner<3, 3>() = 100 * Eigen::Matrix3d::Identity();

    // Effort weight matrix.
    JointMatrix R = JointMatrix::Identity();

    // Lifted error weight matrix is (block) diagonal.
    OptErrorWeightMatrix Qbar = OptErrorWeightMatrix::Zero();
    OptWeightMatrix Rbar = OptWeightMatrix::Zero();
    for (int i = 0; i < NUM_HORIZON; ++i) {
        Qbar.block<6, 6>(i*6, i*6) = Q;
        Rbar.block<NUM_JOINTS, NUM_JOINTS>(i*NUM_JOINTS, i*NUM_JOINTS) = R;
    }

    // E matrix is a lower triangular matrix of ones.
    OptWeightMatrix Ebar = OptWeightMatrix::Ones().triangularView<Eigen::Lower>();

    // Stacked vector of initial joint positions.
    OptVector q0bar = OptVector::Zero();
    for (int i = 0; i < NUM_HORIZON; ++i) {
        q0bar.segment<NUM_JOINTS>(i * NUM_JOINTS) = q0;
    }

    // Current guess for optimal joint positions and velocities.
    OptVector qbar = q0bar;
    OptVector dqbar = OptVector::Zero();
    OptErrorVector ebar = OptErrorVector::Zero();
    OptLiftedJacobian Jbar = OptLiftedJacobian::Zero();

    // TODO Placeholder bounds
    OptVector dq_min = -OptVector::Ones();
    OptVector dq_max = OptVector::Ones();

    // Roll out desired trajectory.
    // TODO handling overshooting the end of the trajectory isn't good
    // because it spreads error for final position over time
    std::vector<Eigen::Affine3d> Tds;
    for (int k = 1; k <= NUM_HORIZON; ++k) {
        double t = t0 + k * LOOKAHEAD_STEP_TIME;

        Eigen::Affine3d Td;
        Vector6d twist;  // unused
        trajectory.sample(t, Td, twist);

        Tds.push_back(Td);
    }

    // Outer loop iterates over linearizations (i.e. QP solves).
    for (int i = 0; i < NUM_ITER; ++i) {
        qbar = q0bar + LOOKAHEAD_STEP_TIME * Ebar * dqbar;

        double t1 = ros::Time::now().toSec();

        // Inner loop iterates over timesteps to build up the matrices.
        // We're abusing indexing notation slightly here: mathematically, we
        // want to iterate from k=1:N, but since indexing is zero-based, it
        // makes more sense to start from zero programmatically. This is in
        // contrast to the above loop rolling out the lookahead trajectory.
        for (int k = 0; k < NUM_HORIZON; ++k) {
            Eigen::Affine3d Td = Tds[k];

            // current guess for joint values k steps in the future
            JointVector qk = qbar.segment<NUM_JOINTS>(NUM_JOINTS * k);

            // calculate pose error
            Vector6d ek;
            pose_error(Td, qk, ek);
            ebar.segment<6>(6 * k) = ek;

            // calculate Jacobian of pose error
            JacobianMatrix Jk;
            pose_error_jacobian(Td, qk, Jk);
            Jbar.block<6, NUM_JOINTS>(6 * k, NUM_JOINTS * k) = Jk;
        }

        double t2 = ros::Time::now().toSec();

        // Construct overall objective matrices.
        OptWeightMatrix H = LOOKAHEAD_STEP_TIME * LOOKAHEAD_STEP_TIME * Ebar.transpose() * Jbar.transpose() * Qbar * Jbar * Ebar + Rbar;
        OptVector g = LOOKAHEAD_STEP_TIME * ebar.transpose() * Qbar * Jbar * Ebar + dqbar.transpose() * Rbar;

        // Bounds
        OptVector lb = dq_min - dqbar;
        OptVector ub = dq_max - dqbar;

        OptVector step = OptVector::Zero();
        solve_sqp(H, g, lb, ub, step);

        double t3 = ros::Time::now().toSec();

        // dqbar updated by new step
        dqbar = dqbar + step;

        // ROS_INFO_STREAM("t12 = " << t2 - t1 << ", t23 = " << t3 - t2);
    }

    dq_opt = dqbar.head<NUM_JOINTS>();
}

} // namespace mm

#include "mm_motion_control/pose_control/mpc_optimizer.h"

#include <Eigen/Eigen>
#include <qpOASES/qpOASES.hpp>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/differentiation.h>

#include "mm_motion_control/pose_control/pose_error.h"
#include "mm_motion_control/pose_control/rate.h"


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

    // ROS_INFO_STREAM("obj " << sqp.getObjVal());

    return qpOASES::getSimpleStatus(ret);
}


int MPCOptimizer::solve(double t0, PoseTrajectory& trajectory,
                        const JointVector& q0, const JointVector& dq0,
                        const std::vector<ObstacleModel>& obstacles,
                        double dt, JointVector& dq_opt) {
    // NOTE: We can run into numerical issues if the weights are too large. Try
    // to balance between downweighting and upweighting between different
    // objectives. This seems to be become more apparent as the horizon grows
    // (so the optimization problem becomes larger).

    // Error weight matrix.
    Matrix6d Q = Matrix6d::Identity();
    Q.topLeftCorner<3, 3>() = 100 * Eigen::Matrix3d::Identity();
    Q.bottomRightCorner<3, 3>() = 0 * Eigen::Matrix3d::Identity();

    // Effort weight matrix.
    JointMatrix R = JointMatrix::Identity();

    // Lifted error weight matrix is (block) diagonal.
    OptErrorWeightMatrix Qbar = OptErrorWeightMatrix::Zero();
    OptWeightMatrix Rbar = OptWeightMatrix::Zero();
    for (int i = 0; i < NUM_HORIZON; ++i) {
        Qbar.block<6, 6>(i*6, i*6) = Q;
        Rbar.block<NUM_JOINTS, NUM_JOINTS>(i*NUM_JOINTS, i*NUM_JOINTS) = R;
    }

    // E matrix is a lower block triangular matrix of identity matrices
    // multiplied by timesteps.
    OptWeightMatrix Ebar = OptWeightMatrix::Zero();
    for (int i = 0; i < NUM_HORIZON; ++i) {
        for (int j = 0; j <= i; ++j) {
            // For the first column of blocks, timestep is the control
            // timestep, otherwise it is the lookahead timestep.
            double timestep = LOOKAHEAD_TIMESTEP;
            if (j == 0) {
                timestep = CONTROL_TIMESTEP;
            }

            // Ebar.block<NUM_JOINTS, NUM_JOINTS>(i*NUM_JOINTS, j*NUM_JOINTS)
            //     = timestep * JointMatrix::Identity();
            Ebar.block<NUM_JOINTS, NUM_JOINTS>(i*NUM_JOINTS, j*NUM_JOINTS)
                = JointMatrix::Identity();
        }
    }

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

    // Velocity bounds.
    OptVector dq_min = OptVector::Zero();
    OptVector dq_max = OptVector::Zero();
    for (int i = 0; i < NUM_HORIZON; ++i) {
        dq_min.segment<NUM_JOINTS>(i * NUM_JOINTS) = VELOCITY_LIMITS_LOWER;
        dq_max.segment<NUM_JOINTS>(i * NUM_JOINTS) = VELOCITY_LIMITS_UPPER;
    }

    // Roll out desired trajectory. The first desired pose is one control
    // timestep into the future (this is the control input for which we wish to
    // solve). Each subsequent one is one lookahead time step into the future,
    // which are generally larger than the control timesteps.
    //
    // TODO handling overshooting the end of the trajectory isn't good
    // because it spreads error for final position over time
    std::vector<Eigen::Affine3d> Tds;
    std::vector<Vector6d> twistds;
    for (int k = 0; k < NUM_HORIZON; ++k) {
        // TODO recall we had the CONTROL_TIMESTEP added in here
        double t = t0 + k * LOOKAHEAD_TIMESTEP;

        Eigen::Affine3d Td;
        Vector6d twistd;
        trajectory.sample(t, Td, twistd);

        Tds.push_back(Td);
        twistds.push_back(twistd);
    }

    int status = 0;

    // Outer loop iterates over linearizations (i.e. QP solves).
    for (int i = 0; i < NUM_ITER; ++i) {
        qbar = q0bar + Ebar * dqbar;

        // Inner loop iterates over timesteps to build up the matrices.
        // We're abusing indexing notation slightly here: mathematically, we
        // want to iterate from k=1:N, but since indexing is zero-based, it
        // makes more sense to start from zero programmatically. This is in
        // contrast to the above loop rolling out the lookahead trajectory.
        for (int k = 0; k < NUM_HORIZON; ++k) {
            Eigen::Affine3d Td = Tds[k];
            Vector6d twistd = twistds[k];

            // current guess for joint values k steps in the future
            JointVector qk = qbar.segment<NUM_JOINTS>(NUM_JOINTS * k);

            // calculate pose error
            Matrix6d Kp = Matrix6d::Identity();
            Vector6d ek;
            pose_error(Td, qk, ek);
            ebar.segment<6>(6 * k) = Kp * ek + twistd;

            // calculate Jacobian of pose error
            JacobianMatrix Jk;
            pose_error_jacobian(Td, qk, Jk);
            Jbar.block<6, NUM_JOINTS>(6 * k, NUM_JOINTS * k) = Jk;
        }

        // Construct overall objective matrices.
        OptWeightMatrix H = Ebar.transpose() * Jbar.transpose() * Qbar * Jbar * Ebar + Rbar;
        OptVector g = ebar.transpose() * Qbar * Jbar * Ebar + dqbar.transpose() * Rbar;

        // Eigen::Matrix<std::complex<double>, NUM_OPT, 1> eivals = H.eigenvalues();
        // ROS_INFO_STREAM("max = " << eivals(0) << ", min = " << eivals(6*NUM_HORIZON-1));

        // Bounds
        OptVector lb = dq_min - dqbar;
        OptVector ub = dq_max - dqbar;

        OptVector step = OptVector::Zero();
        status = solve_sqp(H, g, lb, ub, step);
        if (status) {
            return status;
        }

        // dqbar updated by new step
        dqbar = dqbar + step;
    }

    dq_opt = dqbar.head<NUM_JOINTS>();
    return status;
}

} // namespace mm

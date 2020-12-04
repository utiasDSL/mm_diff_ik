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

    // Error weight matrix.
    // NOTE: We can run into numerical issues if the weights are too large. Try
    // to balance between downweighting and upweighting between different
    // objectives. This seems to be become more apparent as the horizon grows
    // (so the optimization problem becomes larger).
    Matrix6d Q = Matrix6d::Identity();
    Q.topLeftCorner<3, 3>() = 10 * Eigen::Matrix3d::Identity();
    Q.bottomRightCorner<3, 3>() = 0 * Eigen::Matrix3d::Identity();

    // Effort weight matrix.
    JointMatrix R = JointMatrix::Identity();

    // Lifted error weight matrix is (block) diagonal.
    Qbar = OptErrorWeightMatrix::Zero();
    Rbar = OptWeightMatrix::Zero();
    for (int i = 0; i < NUM_HORIZON; ++i) {
        Qbar.block<6, 6>(i*6, i*6) = Q;
        Rbar.block<NUM_JOINTS, NUM_JOINTS>(i*NUM_JOINTS, i*NUM_JOINTS) = R;
    }

    // E matrix is a lower block triangular matrix of identity matrices
    // multiplied by timesteps.
    Ebar = OptWeightMatrix::Zero();
    for (int i = 0; i < NUM_HORIZON; ++i) {
        for (int j = 0; j <= i; ++j) {
            Ebar.block<NUM_JOINTS, NUM_JOINTS>(i*NUM_JOINTS, j*NUM_JOINTS)
                = JointMatrix::Identity();
        }
    }

    // Velocity and acceleration limits.
    dq_min = OptVector::Zero();
    dq_max = OptVector::Zero();
    ddq_min = OptVector::Zero();
    ddq_max = OptVector::Zero();

    for (int i = 0; i < NUM_HORIZON; ++i) {
        dq_min.segment<NUM_JOINTS>(i * NUM_JOINTS) = VELOCITY_LIMITS_LOWER;
        dq_max.segment<NUM_JOINTS>(i * NUM_JOINTS) = VELOCITY_LIMITS_UPPER;
        ddq_min.segment<NUM_JOINTS>(i * NUM_JOINTS) = ACCEL_LIMITS_LOWER;
        ddq_max.segment<NUM_JOINTS>(i * NUM_JOINTS) = ACCEL_LIMITS_UPPER;
    }

    // A has identity matrices on the main block diagonal, and negative
    // identity matrices on the lower block diagonal
    A = OptWeightMatrix::Identity();
    Eigen::Matrix<double, NUM_OPT - NUM_JOINTS, NUM_OPT - NUM_JOINTS> A_1 = -Eigen::Matrix<double, NUM_OPT - NUM_JOINTS, NUM_OPT - NUM_JOINTS>::Identity();
    A.block<NUM_OPT - NUM_JOINTS, NUM_OPT - NUM_JOINTS>(NUM_JOINTS, 0) += A_1;

    return true;
}


int MPCOptimizer::solve_sqp(OptWeightMatrix& H, OptVector& g, OptVector& lb,
                            OptVector& ub, OptWeightMatrix& A, OptVector& lbA,
                            OptVector& ubA, OptVector& du) {
    using OptWeightMatrixRM = Eigen::Matrix<qpOASES::real_t, NUM_OPT, NUM_OPT,
                                            Eigen::RowMajor>;

    // Convert to row-major order. Only meaningful for matrices, not
    // vectors.
    OptWeightMatrixRM H_rowmajor = H;
    OptWeightMatrixRM A_rowmajor = A;

    // Convert eigen data to raw arrays for qpOASES.
    qpOASES::real_t *H_data = H_rowmajor.data();
    qpOASES::real_t *g_data = g.data();
    qpOASES::real_t *A_data = A_rowmajor.data();

    qpOASES::real_t *lb_data = lb.data();
    qpOASES::real_t *ub_data = ub.data();

    qpOASES::real_t *lbA_data = lbA.data();
    qpOASES::real_t *ubA_data = ubA.data();

    qpOASES::int_t nWSR = NUM_WSR;

    // Solve the QP.
    qpOASES::returnValue ret;
    if (sqp.isInitialised()) {
        ret = sqp.hotstart(H_data, g_data, A_data, lb_data, ub_data, lbA_data, ubA_data, nWSR);
    } else {
        ret = sqp.init(H_data, g_data, A_data, lb_data, ub_data, lbA_data, ubA_data, nWSR);
    }

    qpOASES::real_t du_data[NUM_OPT];
    sqp.getPrimalSolution(du_data);
    du = Eigen::Map<OptVector>(du_data); // map back to eigen

    return qpOASES::getSimpleStatus(ret);
}


int MPCOptimizer::solve(double t0, PoseTrajectory& trajectory,
                        const JointVector& q0, const JointVector& dq0,
                        const std::vector<ObstacleModel>& obstacles,
                        double dt, JointVector& u) {
    // Stacked vector of initial joint positions.
    OptVector q0bar = OptVector::Zero();
    for (int i = 0; i < NUM_HORIZON; ++i) {
        q0bar.segment<NUM_JOINTS>(i * NUM_JOINTS) = q0;
    }

    // Current guess for optimal joint positions and velocities.
    OptVector qbar = q0bar;
    OptVector ubar = OptVector::Zero();
    OptErrorVector ebar = OptErrorVector::Zero();
    OptLiftedJacobian Jbar = OptLiftedJacobian::Zero();

    // Roll out desired trajectory. The first desired pose is one control
    // timestep into the future (this is the control input for which we wish to
    // solve). Each subsequent one is one lookahead time step into the future,
    // which are generally larger than the control timesteps.
    std::vector<Eigen::Affine3d> Tds;
    for (int k = 1; k <= NUM_HORIZON; ++k) {
        double t = t0 + k * LOOKAHEAD_TIMESTEP;

        Eigen::Affine3d Td;
        Vector6d Vd;
        trajectory.sample(t, Td, Vd);

        Tds.push_back(Td);
    }

    int status = 0;

    // Outer loop iterates over linearizations (i.e. QP solves).
    for (int i = 0; i < NUM_ITER; ++i) {
        // Integrate to get joint positions.
        qbar = q0bar + LOOKAHEAD_TIMESTEP * Ebar * ubar;

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
            Matrix6d Kp = Matrix6d::Identity();
            Vector6d ek;
            pose_error(Td, qk, ek);
            ebar.segment<6>(6 * k) = ek;

            // calculate Jacobian of pose error
            JacobianMatrix Jk;
            pose_error_jacobian(Td, qk, Jk);
            Jbar.block<6, NUM_JOINTS>(6 * k, NUM_JOINTS * k) = Jk;
        }

        // Construct overall objective matrices.
        OptWeightMatrix H = LOOKAHEAD_TIMESTEP * LOOKAHEAD_TIMESTEP * Ebar.transpose() * Jbar.transpose() * Qbar * Jbar * Ebar + Rbar;
        OptVector g = LOOKAHEAD_TIMESTEP * ebar.transpose() * Qbar * Jbar * Ebar + ubar.transpose() * Rbar;

        // Velocity bounds
        OptVector lb = dq_min - ubar;
        OptVector ub = dq_max - ubar;

        // Acceleration constraints
        OptVector ubar_1;
        ubar_1 << dq0, ubar.head<NUM_OPT - NUM_JOINTS>();
        OptVector nu = ubar - ubar_1;
        OptVector lbA = LOOKAHEAD_TIMESTEP * ddq_min - nu;
        OptVector ubA = LOOKAHEAD_TIMESTEP * ddq_max - nu;

        OptVector du = OptVector::Zero();
        status = solve_sqp(H, g, lb, ub, A, lbA, ubA, du);
        if (status) {
            return status;
        }

        // ubar updated by new step
        ubar = ubar + du;
    }

    u = ubar.head<NUM_JOINTS>();
    return status;
}

} // namespace mm

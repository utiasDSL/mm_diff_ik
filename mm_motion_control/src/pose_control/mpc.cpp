#include "mm_motion_control/pose_control/mpc.h"

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <qpOASES/qpOASES.hpp>

#include <mm_optimization/qpoases.h>
#include <mm_kinematics/kinematics.h>


namespace mm {


bool MPController::init(ros::NodeHandle& nh, const double hz) {
    mm::CartesianController::init(nh, hz);

    sqp.options.printLevel = qpOASES::PL_LOW;

    // Error weight matrix.
    // NOTE: We can run into numerical issues if the weights are too large. Try
    // to balance between downweighting and upweighting between different
    // objectives. This seems to be become more apparent as the horizon grows
    // (so the optimization problem becomes larger).
    Matrix6d Q = Matrix6d::Identity();
    Q.topLeftCorner<3, 3>() = 100 * Eigen::Matrix3d::Identity();
    Q.bottomRightCorner<3, 3>() = 100 * Eigen::Matrix3d::Identity();

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


int MPController::update(const ros::Time& now) {
    // double t0 = now.toSec();

    // Stacked vector of initial joint positions.
    OptVector q0bar = OptVector::Zero();
    for (int i = 0; i < NUM_HORIZON; ++i) {
        q0bar.segment<NUM_JOINTS>(i * NUM_JOINTS) = q;
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
    // std::vector<Pose> Pds;
    // for (int k = 1; k <= NUM_HORIZON; ++k) {
    //     ros::Time sample_time = now + ros::Duration(k * LOOKAHEAD_TIMESTEP);
    //
    //     CartesianPosVelAcc Xd;
    //     trajectory.sample(sample_time, Xd);
    //     Pds.push_back(Xd.pose);
    // }

    std::vector<ros::Time> sample_times;
    for (int k = 1; k <= NUM_HORIZON; ++k) {
        sample_times.push_back(now + ros::Duration(k * LOOKAHEAD_TIMESTEP));
    }
    std::vector<CartesianPosVelAcc> Xds;
    trajectory.sample(now, sample_times, Xds);

    int status = 0;

    // Outer loop iterates over linearizations (i.e. QP solves).
    for (int i = 0; i < NUM_ITER; ++i) {
        // Integrate to get joint positions.
        // TODO this is not right due to B(q) being introduced: need to rewrite
        // to accommodate this
        qbar = q0bar + LOOKAHEAD_TIMESTEP * Ebar * ubar;

        // Inner loop iterates over timesteps to build up the matrices.
        // We're abusing indexing notation slightly here: mathematically, we
        // want to iterate from k=1:N, but since indexing is zero-based, it
        // makes more sense to start from zero programmatically. This is in
        // contrast to the above loop rolling out the lookahead trajectory.
        for (int k = 0; k < NUM_HORIZON; ++k) {
            Pose Pd = Xds[k].pose;

            // current guess for joint values k steps in the future
            JointVector qk = qbar.segment<NUM_JOINTS>(NUM_JOINTS * k);

            // calculate pose error
            Vector6d ek;
            calc_cartesian_control_error(Pd, qk, ek);
            ebar.segment<6>(6 * k) = ek;

            // calculate Jacobian and B to map inputs to generalized velocities
            JointMatrix Bk;
            JacobianMatrix Jk;
            Kinematics::calc_joint_input_map(qk, Bk);
            Kinematics::jacobian(qk, Jk);
            JacobianMatrix JBk = Jk * Bk;
            Jbar.block<6, NUM_JOINTS>(6 * k, NUM_JOINTS * k) = JBk;
        }

        // Construct overall objective matrices.
        OptWeightMatrix H = LOOKAHEAD_TIMESTEP * LOOKAHEAD_TIMESTEP * Ebar.transpose() * Jbar.transpose() * Qbar * Jbar * Ebar + Rbar;
        OptVector g = -LOOKAHEAD_TIMESTEP * ebar.transpose() * Qbar * Jbar * Ebar + ubar.transpose() * Rbar;

        // Velocity bounds
        OptVector lb = dq_min - ubar;
        OptVector ub = dq_max - ubar;

        // Acceleration constraints
        OptVector ubar_1;
        ubar_1 << dq, ubar.head<NUM_OPT - NUM_JOINTS>();
        OptVector nu = ubar - ubar_1;
        OptVector lbA = LOOKAHEAD_TIMESTEP * ddq_min - nu;
        OptVector ubA = LOOKAHEAD_TIMESTEP * ddq_max - nu;

        sqp.data.H = H;
        sqp.data.g = g;
        sqp.data.A = A;
        sqp.data.lb = lb;
        sqp.data.ub = ub;
        sqp.data.lbA = lbA;
        sqp.data.ubA = ubA;

        Eigen::VectorXd du;
        status = sqp.solve(du);
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

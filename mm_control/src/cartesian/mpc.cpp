#include "mm_control/cartesian/mpc.h"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <qpOASES/qpOASES.hpp>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/rotation.h>
#include <mm_optimization/qpoases.h>

#include <mm_control/cartesian/point.h>

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
    Qbar.block<6, 6>(i * 6, i * 6) = Q;
    Rbar.block<NUM_JOINTS, NUM_JOINTS>(i * NUM_JOINTS, i * NUM_JOINTS) = R;
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
  Eigen::Matrix<double, NUM_OPT - NUM_JOINTS, NUM_OPT - NUM_JOINTS> A_1 =
      -Eigen::Matrix<double, NUM_OPT - NUM_JOINTS,
                     NUM_OPT - NUM_JOINTS>::Identity();
  A.block<NUM_OPT - NUM_JOINTS, NUM_OPT - NUM_JOINTS>(NUM_JOINTS, 0) += A_1;

  return true;
}

int MPController::update(const ros::Time& now) {
  // Current guess for optimal joint positions and velocities.
  OptVector ubar = OptVector::Zero();
  OptErrorVector ebar = OptErrorVector::Zero();
  OptLiftedJacobian Jbar = OptLiftedJacobian::Zero();

  // Roll out desired trajectory. The first desired pose is one control
  // timestep into the future (this is the control input for which we wish to
  // solve). Each subsequent one is one lookahead time step into the future,
  // which are generally larger than the control timesteps.
  std::vector<ros::Time> sample_times;
  for (int k = 1; k <= NUM_HORIZON; ++k) {
    sample_times.push_back(now + ros::Duration(k * LOOKAHEAD_TIMESTEP));
  }
  std::vector<CartesianTrajectoryPoint> Xds;
  trajectory.sample(now, sample_times, Xds);

  int status = 0;

  // Outer loop iterates over linearizations (i.e. QP solves).
  for (int i = 0; i < NUM_ITER; ++i) {
    JointVector qk = q;
    OptWeightMatrix Fbar = OptWeightMatrix::Zero();

    // Inner loop iterates over timesteps to build up the matrices.
    // We're abusing indexing notation slightly here: mathematically, we
    // want to iterate from k=1:N, but since indexing is zero-based, it
    // makes more sense to start from zero programmatically. This is in
    // contrast to the above loop rolling out the lookahead trajectory.
    for (int k = 0; k < NUM_HORIZON; ++k) {
      JointVector uk = ubar.segment<NUM_JOINTS>(NUM_JOINTS * k);
      JointMatrix Bk;
      Kinematics::calc_joint_input_map(qk, Bk);

      // Add entries to the linearized dynamics matrix
      // k-th column is handled separately: it is derivate of q_k w.r.t.
      // u_{k-1}
      Fbar.block(k * NUM_JOINTS, k * NUM_JOINTS, (NUM_HORIZON - k) * NUM_JOINTS,
                 NUM_JOINTS) =
          LOOKAHEAD_TIMESTEP * Bk.replicate(NUM_HORIZON - k, 1);

      // Columns before the k-th: derivative of q_k w.r.t. q_{k-1}
      JointMatrix M = JointMatrix::Identity();
      M.block<2, 1>(0, 2) =
          LOOKAHEAD_TIMESTEP * rotation2d_derivative(qk(2)) * uk.head<2>();
      for (int c = 0; c < k; ++c) {
        for (int r = k; r < NUM_HORIZON; ++r) {
          Fbar.block<NUM_JOINTS, NUM_JOINTS>(NUM_JOINTS * r, NUM_JOINTS * c)
              .applyOnTheLeft(M);
        }
      }

      // Integrate motion model forward using previous optimal inputs
      qk = qk + LOOKAHEAD_TIMESTEP * Bk * uk;

      // Calculate Cartesian pose error
      ebar.segment<6>(6 * k) = calc_cartesian_control_error(Xds[k].pose, qk);

      // Calculate Jacobian
      JacobianMatrix Jk;
      Kinematics::jacobian(qk, Jk);
      Jbar.block<6, NUM_JOINTS>(6 * k, NUM_JOINTS * k) = Jk;
    }

    // Construct overall objective matrices.
    OptWeightMatrix H =
        Fbar.transpose() * Jbar.transpose() * Qbar * Jbar * Fbar + Rbar;
    OptVector g =
        -ebar.transpose() * Qbar * Jbar * Fbar + ubar.transpose() * Rbar;

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
    // TODO we could do a line search here
    ubar = ubar + du;
  }

  u = ubar.head<NUM_JOINTS>();
  return status;
}

}  // namespace mm

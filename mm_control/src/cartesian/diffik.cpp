#include "mm_control/cartesian/diffik.h"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <qpOASES/qpOASES.hpp>

#include <mm_msgs/Obstacles.h>
#include <mm_msgs/WrenchInfo.h>
#include <std_msgs/Float64.h>

#include <mm_kinematics/kinematics.h>
#include <mm_kinematics/spatial.h>
#include <mm_math_util/interp.h>
#include <mm_optimization/qpoases.h>

#include <mm_control/cartesian/obstacle.h>
#include <mm_control/cartesian/point.h>

namespace mm {

bool DiffIKController::init(ros::NodeHandle& nh, const double hz) {
  mm::CartesianController::init(nh, hz);

  wrench_info_sub = nh.subscribe("/mm/force_torque/info", 1,
                                 &DiffIKController::wrench_info_cb, this);

  // TODO redundant: also contained in force info
  force_des_sub = nh.subscribe("/mm/force_torque/desired", 1,
                               &DiffIKController::force_des_cb, this);

  obstacle_sub =
      nh.subscribe("/mm/obstacles", 1, &DiffIKController::obstacle_cb, this);

  fd = 0;
  force = Eigen::Vector3d::Zero();
  torque = Eigen::Vector3d::Zero();
  nf_xy << 1, 0;

  return true;
}

int DiffIKController::update(const ros::Time& now) {
  // Integrate using the model to get the best estimate of the state.
  // TODO: not sure if it would be better to use u instead of dq
  // -> would be less noisy
  // double dt = t - last_joint_state_time;
  // q = q + dt * dq;
  // last_joint_state_time = t;

  // Primary objective function.
  JointMatrix H;
  JointVector g;
  calc_primary_objective(now, H, g);

  // Velocity damper inequality constraints on joints.
  JointVector u_lb, u_ub;
  calc_joint_limits(u_lb, u_ub);

  // Obstacle constraints.
  Eigen::MatrixXd A_obs;
  Eigen::VectorXd ub_obs;
  calc_obstacle_constraints(A_obs, ub_obs);

  // Setup and solve the first QP.
  // TODO handle state
  qpoases::QProblem qp(NUM_JOINTS, A_obs.rows(), NUM_WSR);
  qp.options.printLevel = qpOASES::PL_LOW;

  qp.data.H = H;
  qp.data.g = g;

  qp.data.lb = u_lb;
  qp.data.ub = u_ub;

  qp.data.A = A_obs;
  qp.data.ubA = ub_obs;
  qp.data.lbA.setConstant(A_obs.rows(), -qpOASES::INFTY);

  // Second QP can optimize manipulability.
  Eigen::VectorXd u1, u2;
  int status1 = qp.solve(u1);
  int status2 = 0;  // 0 = good
  u = u1;

  // uncomment to solve manipulability objective in the nullspace of primary QP
  // solution
  // status2 = nullspace_manipulability(qp.data, u1, u2);
  // u = u2;

  // If either status is non-zero, final status is too.
  return status1 || status2;
}

void DiffIKController::wrench_info_cb(const mm_msgs::WrenchInfo& msg) {
  force << msg.world.force.x, msg.world.force.y, msg.world.force.z;
  torque << msg.world.torque.x, msg.world.torque.y, msg.world.torque.z;
  // fd = msg.force_desired
}

void DiffIKController::force_des_cb(const std_msgs::Float64 msg) {
  fd = msg.data;
  ROS_INFO_STREAM("Set desired force = " << fd);
}

void DiffIKController::obstacle_cb(const mm_msgs::Obstacles& msg) {
  obstacles.clear();
  for (int i = 0; i < msg.obstacles.size(); ++i) {
    obstacles.push_back(ObstacleModel(msg.obstacles[i]));
  }
}

int DiffIKController::nullspace_manipulability(qpoases::QPData& qp1_data,
                                               const Eigen::VectorXd& u1,
                                               Eigen::VectorXd& u2) {
  JacobianMatrix J;
  JointMatrix B;
  Kinematics::jacobian(q, J);
  Kinematics::calc_joint_input_map(q, B);

  JacobianMatrix A_eq = J * B;
  Vector6d b_eq = A_eq * u1;

  JointVector m_grad;
  Kinematics::manipulability_gradient_analytic(q, m_grad);
  double w_mi = 1;
  JointVector g = -w_mi * dt * B.transpose() * m_grad;

  // Add the equality constraints.
  Eigen::MatrixXd A(A_eq.rows() + qp1_data.A.rows(), A_eq.cols());
  A << qp1_data.A, A_eq;

  Eigen::VectorXd lbA(A.rows());
  lbA << qp1_data.lbA, b_eq;

  Eigen::VectorXd ubA(A.rows());
  ubA << qp1_data.ubA, b_eq;

  qpoases::QProblem qp(NUM_JOINTS, A.rows(), NUM_WSR);
  qp.options.setToReliable();
  qp.options.printLevel = qpOASES::PL_LOW;

  // Reuse the data from the first QP, except where we need to make changes.
  qp.data = qp1_data;
  qp.data.H = JointMatrix::Identity();
  qp.data.g = g;

  qp.data.A = A;
  qp.data.lbA = lbA;
  qp.data.ubA = ubA;

  return qp.solve(u2);
}

void DiffIKController::minimize_joint_velocity_objective(const JointMatrix& Q,
                                                         JointMatrix& H,
                                                         JointVector& g) {
  H = Q;
  g = JointVector::Zero();
}

void DiffIKController::minimize_joint_acceleration_objective(JointMatrix& H,
                                                             JointVector& g) {
  H = JointMatrix::Zero();
  g = JointVector::Zero();

  // Only do numerical differentiation with a non-zero timestep.
  if (dt * dt > 0) {
    H = JointMatrix::Identity() / (dt * dt);
    g = -dq.transpose() * H;
  }
}

void DiffIKController::avoid_joint_limits_objective(JointMatrix& H,
                                                    JointVector& g) {
  // Base has no limits, so first three joints are unweighted.
  H = dt * dt * JointMatrix::Identity();
  H.topLeftCorner<3, 3>() = Eigen::Matrix3d::Zero();

  g = dt * (2 * q - (POSITION_LIMITS_UPPER + POSITION_LIMITS_LOWER));
  g.head<3>() = Eigen::Vector3d::Zero();
}

void DiffIKController::pose_error_and_force_compliance_objective(
    const ros::Time& now,
    const Matrix6d& Kp,
    const Matrix6d& Kf,
    const Matrix6d& W,
    JointMatrix& H,
    JointVector& g) {
  // Sample the trajectory.
  CartesianTrajectoryPoint Xd;
  trajectory.sample(now, Xd);

  Vector6d Vd;
  Vd << Xd.twist.linear, Xd.twist.angular;

  // Calculate mapping from base inputs to generalized coordinates. The
  // former are in the base frame while the latter are in the world frame.
  JointMatrix B = JointMatrix::Identity();
  Kinematics::calc_joint_input_map(q, B);

  // Calculate Jacobian.
  JacobianMatrix J;
  Kinematics::jacobian(q, J);
  JacobianMatrix JB = J * B;

  Eigen::Vector3d f_compliant = force;
  Eigen::Vector3d torque_compliant = torque;

  double f_norm = force.norm();
  if (f_norm > MAX_COMPLIANCE_FORCE) {
    f_compliant = MAX_COMPLIANCE_FORCE * force / f_norm;
  }

  Vector6d wrench_compliant;
  wrench_compliant << f_compliant, torque_compliant;

  Vector6d wrench_compliant_d = Vector6d::Zero();
  wrench_compliant_d << 0, 0, fd, 0, 0, 0;

  Vector6d P_err = calc_cartesian_control_error(Xd.pose, q);
  Vector6d W_err = wrench_compliant_d - wrench_compliant;

  Vector6d V_compliant = Vd + Kp * P_err + Kf * W_err;

  H = JB.transpose() * W * JB;
  g = -V_compliant.transpose() * W * JB;
}

void DiffIKController::pushing_objective(const ros::Time& now,
                                         const double alpha_push,
                                         const double alpha_nf,
                                         const Matrix6d& W,
                                         JointMatrix& H,
                                         JointVector& g) {
  // Sample the trajectory.
  CartesianTrajectoryPoint Xd;
  trajectory.sample(now, Xd);

  // Calculate mapping from base inputs to generalized coordinates. The
  // former are in the base frame while the latter are in the world frame.
  JointMatrix B = JointMatrix::Identity();
  Kinematics::calc_joint_input_map(q, B);

  // Calculate pose error.
  Vector6d P_err = calc_cartesian_control_error(Xd.pose, q);

  // Calculate Jacobian.
  JacobianMatrix J;
  Kinematics::jacobian(q, J);
  JacobianMatrix JB = J * B;

  // position normal
  Eigen::Vector2d np_xy = P_err.head<2>().normalized();

  // nf_xy decays back to the push direction in the absence of sufficiently
  // large force measurements (i.e. ones that aren't noise)
  // NOTE: approach is sensitive to noisy force measurements if above the
  // threshold: they will cause jerky behaviour, because the controller
  // thinks it is in contact with the object
  Eigen::Vector2d f_xy = force.head<2>();
  if (f_xy.norm() > FORCE_THRESHOLD) {
    nf_xy = f_xy.normalized();
  } else {
    nf_xy = slerp2d(nf_xy, np_xy, alpha_nf);
  }

  // push direction balances between the two
  Eigen::Vector2d push_dir = slerp2d(nf_xy, np_xy, -alpha_push);

  // TODO fixed gains for now
  Vector6d V_push = Vector6d::Zero();
  V_push << 0.2 * push_dir, 1 * P_err.tail<4>();

  H = JB.transpose() * W * JB;
  g = -V_push.transpose() * W * JB;

  // for safety, in case end of trajectory is reached
  // if (P_err.head<2>().norm() <= 0.2) {
  //     ROS_INFO_STREAM("near end, stopping");
  //     H = JointMatrix::Zero();
  //     g = JointVector::Zero();
  // }
}

void DiffIKController::manipulability_objective(JointMatrix& H,
                                                JointVector& g) {
  JointMatrix B = JointMatrix::Identity();
  Kinematics::calc_joint_input_map(q, B);

  JointVector m_grad;
  Kinematics::manipulability_gradient_analytic(q, m_grad);
  g = -dt * B.transpose() * m_grad;
  H = JointMatrix::Zero();
}

void DiffIKController::calc_primary_objective(const ros::Time& now,
                                              JointMatrix& H,
                                              JointVector& g) {
  /*************************************************************************/
  /* Minimize velocity objective. */

  JointMatrix R = JointMatrix::Identity();
  // To reduce base movement, increase the base joint weighting.
  // R.topLeftCorner<3, 3>() = 10 * Eigen::Matrix3d::Identity();

  JointMatrix H_vel;
  JointVector g_vel;
  minimize_joint_velocity_objective(R, H_vel, g_vel);

  /*************************************************************************/
  /* Avoid joint limits objective. */

  JointMatrix H_limit;
  JointVector g_limit;
  avoid_joint_limits_objective(H_limit, g_limit);

  /*************************************************************************/
  /* Minimize joint acceleration */

  JointMatrix H_acc;
  JointVector g_acc;
  minimize_joint_acceleration_objective(H_acc, g_acc);

  /*************************************************************************/
  /* Pose error and force compliance */

  Matrix6d Kp = Matrix6d::Identity();
  // Kp.diagonal() << 1, 1, 1, 0, 0, 0;
  // Kp.diagonal() << 1, 1, 0, 1, 1, 1;

  // only comply in directions where gain is non-zero
  // 0.01 is pretty good for human interaction; could even probably go a bit
  // higher
  Matrix6d Kf = 0.01 * Matrix6d::Identity();
  // Kf.diagonal() << 0, 0, 0.005, 0, 0, 0;

  Matrix6d W = Matrix6d::Identity();

  JointMatrix H_err;
  JointVector g_err;
  pose_error_and_force_compliance_objective(now, Kp, Kf, W, H_err, g_err);

  /*************************************************************************/
  /* Pushing */

  JointMatrix H_push;
  JointVector g_push;
  Matrix6d W_push = Matrix6d::Identity();
  pushing_objective(now, /* alpha_push = */ 0.5, /* alpha_nf = */ 0.01, W_push,
                    H_push, g_push);

  /*************************************************************************/
  /* Manipulability */

  JointMatrix H_manip;
  JointVector g_manip;
  manipulability_objective(H_manip, g_manip);

  /*************************************************************************/
  /* Objective weighting */

  double w_vel = 1.0;  // should typically be 1.0
  double w_limit = 0.0;
  double w_acc = 0.0;
  double w_err = 100.0;
  double w_push = 0.0;
  double w_manip = 0.0;

  H = w_vel * H_vel + w_limit * H_limit + w_acc * H_acc + w_err * H_err +
      w_push * H_push + w_manip * H_manip;
  g = w_vel * g_vel + w_limit * g_limit + w_acc * g_acc + w_err * g_err +
      w_push * g_push + w_manip * g_manip;
}

void DiffIKController::calc_joint_limits(JointVector& lb, JointVector& ub) {
  // TODO does this handle base input mapping?
  for (int i = 0; i < NUM_JOINTS; ++i) {
    // Upper bound
    double del_q_ub = POSITION_LIMITS_UPPER(i) - q(i);
    ub(i) = VELOCITY_LIMITS_UPPER(i);

    if (POSITIION_LIMITED[i] && del_q_ub <= INFLUENCE_DIST(i)) {
      ROS_WARN_STREAM("Joint " << JOINT_NAMES[i]
                               << " within influence distance of upper bound.");
      ub(i) *=
          (del_q_ub - SAFETY_DIST(i)) / (INFLUENCE_DIST(i) - SAFETY_DIST(i));
    }

    // Lower bound
    double del_q_lb = q(i) - POSITION_LIMITS_LOWER(i);
    lb(i) = VELOCITY_LIMITS_LOWER(i);

    if (POSITIION_LIMITED[i] && del_q_lb <= INFLUENCE_DIST(i)) {
      ROS_WARN_STREAM("Joint " << JOINT_NAMES[i]
                               << " within influence distance of lower bound.");
      lb(i) *=
          (del_q_lb - SAFETY_DIST(i)) / (INFLUENCE_DIST(i) - SAFETY_DIST(i));
    }
  }
}

void filter_obstacles(const Eigen::Vector2d& pb,
                      const std::vector<ObstacleModel>& obstacles,
                      std::vector<ObstacleModel>& close_obstacles) {
  for (int i = 0; i < obstacles.size(); ++i) {
    if (obstacles[i].in_range(pb)) {
      close_obstacles.push_back(obstacles[i]);
    }
  }
}

int DiffIKController::calc_obstacle_constraints(Eigen::MatrixXd& A,
                                                Eigen::VectorXd& b) {
  // Only consider obstacles within the influence distance.
  Eigen::Vector2d pb(q[0], q[1]);
  std::vector<ObstacleModel> close_obstacles;
  filter_obstacles(pb, obstacles, close_obstacles);

  int num_obs = close_obstacles.size();
  A.resize(num_obs, NUM_JOINTS);
  b.resize(num_obs);

  // Start with all coefficients as zero, though not really necessary
  // for b.
  A.setZero();
  b.setZero();

  for (int i = 0; i < close_obstacles.size(); ++i) {
    Eigen::Vector2d n = close_obstacles[i].centre() - pb;
    double d = n.norm() - BASE_RADIUS - close_obstacles[i].radius();
    n.normalize();

    double limit = OBS_COEFF * (d - OBS_SAFETY_DIST) /
                   (OBS_INFLUENCE_DIST - OBS_SAFETY_DIST);

    A(i, 0) = n(0);
    A(i, 1) = n(1);
    b(i) = limit;
  }

  return num_obs;
}

}  // namespace mm

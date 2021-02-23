#include "mm_motion_control/pose_control/optimizer.h"

#include <Eigen/Eigen>
#include <qpOASES/qpOASES.hpp>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/differentiation.h>
#include <mm_math_util/interp.h>

#include "mm_motion_control/pose_control/obstacle.h"
#include "mm_motion_control/pose_control/pose_error.h"
#include "mm_motion_control/pose_control/rate.h"


namespace mm {


static const int NUM_WSR = 50;


bool IKOptimizer::init() {
    nf << 1, 0, 0;
    nf_xy << 1, 0;
    freaked_out = false;
    did_print_gs = false;
    did_print_once = false;
    return true;
}

int IKOptimizer::solve_qp(JointMatrix& H, JointVector& g, Eigen::MatrixXd& A,
                          JointVector& lb, JointVector& ub,
                          Eigen::VectorXd& ubA, JointVector& u) {
    // Convert to row-major order. Only meaningful for matrices, not
    // vectors.
    Eigen::Matrix<qpOASES::real_t,
                  NUM_JOINTS,
                  NUM_JOINTS,
                  Eigen::RowMajor> H_rowmajor = H;
    Eigen::Matrix<qpOASES::real_t,
                  Eigen::Dynamic,
                  Eigen::Dynamic,
                  Eigen::RowMajor> A_rowmajor = A;

    // disable base
    // lb.head<3>() = Eigen::Vector3d::Zero();
    // ub.head<3>() = Eigen::Vector3d::Zero();

    // disable arm
    // lb.tail<6>() = Vector6d::Zero();
    // ub.tail<6>() = Vector6d::Zero();

    // Convert eigen data to raw arrays for qpOASES.
    qpOASES::real_t *H_data = H_rowmajor.data();
    qpOASES::real_t *g_data = g.data();

    qpOASES::real_t *lb_data = lb.data();
    qpOASES::real_t *ub_data = ub.data();

    qpOASES::real_t *A_data = A_rowmajor.data();
    qpOASES::real_t *ubA_data = ubA.data();
    qpOASES::real_t *lbA_data = NULL;

    qpOASES::int_t nWSR = NUM_WSR;

    int num_obstacles = A.rows();

    // Solve the QP.
    // NOTE: cannot warm start if we're using a variable number of constraints
    // (obstacles) at each time step.
    qpOASES::QProblem qp(NUM_JOINTS, num_obstacles);
    qp.setPrintLevel(qpOASES::PL_LOW);  // only error messages
    qpOASES::returnValue ret = qp.init(H_data, g_data, A_data, lb_data, ub_data,
                                       lbA_data, ubA_data, nWSR);
    int status = qpOASES::getSimpleStatus(ret);

    qpOASES::real_t x[NUM_JOINTS];
    qp.getPrimalSolution(x);
    u = Eigen::Map<JointVector>(x); // map back to eigen

    // Populate state values;
    state.x = u;
    state.obj_val = qp.getObjVal();
    state.code = ret;
    state.status = status;

    if (isnan(state.obj_val) || isinf(state.obj_val)) {
        if (!freaked_out) {
            ROS_WARN_STREAM("Objective value = " << state.obj_val);
            ROS_WARN_STREAM("u = " << u);
            ROS_WARN_STREAM("ret = " << ret);
            ROS_WARN_STREAM("status = " << status);
            ROS_WARN_STREAM("num_obs = " << num_obstacles);
            ROS_WARN_STREAM("H = " << H);
            ROS_WARN_STREAM("g = " << g);
            ROS_WARN_STREAM("lb = " << lb);
            ROS_WARN_STREAM("ub = " << ub);

            qp.printProperties();
            qp.printOptions();
            freaked_out = true;
        }

        u = JointVector::Zero();
    }

    // TODO redundant for now, but want to be safe
    if (freaked_out) {
        u = JointVector::Zero();
    }

    return status;
}


int solve_qp_mi(JointVector& g, JointVector& lb, JointVector& ub,
                JacobianMatrix& A_eq, Vector6d& b_eq, Eigen::MatrixXd& A_obs,
                Eigen::VectorXd& ub_obs, JointVector& u) {
    using QPMatrixX = Eigen::Matrix<qpOASES::real_t,
                                    Eigen::Dynamic,
                                    Eigen::Dynamic,
                                    Eigen::RowMajor>;

    // For stability
    JointMatrix H = JointMatrix::Identity();

    QPMatrixX A(A_eq.rows() + A_obs.rows(), A_eq.cols());
        A << A_obs, A_eq;

    Eigen::VectorXd lb_obs(A_obs.rows());
    lb_obs.setConstant(A_obs.rows(), -qpOASES::INFTY);
    Eigen::VectorXd lbA(A.rows());
        lbA << lb_obs, b_eq;

    Eigen::VectorXd ubA(A.rows());
        ubA << ub_obs, b_eq;

    // Convert eigen data to raw arrays for qpOASES.
    qpOASES::real_t *H_data = H.data();
    qpOASES::real_t *g_data = g.data();

    qpOASES::real_t *lb_data = lb.data();
    qpOASES::real_t *ub_data = ub.data();

    // Equality constraint.
    qpOASES::real_t *A_data = A.data();
    qpOASES::real_t *lbA_data = lbA.data();
    qpOASES::real_t *ubA_data = ubA.data();

    qpOASES::int_t nWSR = NUM_WSR;

    // Setting options to reliable seems to be necessary for the solver to
    // work for this problem.
    qpOASES::Options options;
    options.setToReliable();
    options.printLevel = qpOASES::PL_LOW;

    // Solve the QP.
    qpOASES::QProblem qp(NUM_JOINTS, A.rows());
    qp.setOptions(options);
    qpOASES::returnValue ret = qp.init(H_data, g_data, A_data, lb_data, ub_data,
                                       lbA_data, ubA_data, nWSR);
    int status = qpOASES::getSimpleStatus(ret);

    qpOASES::real_t x[NUM_JOINTS];
    qp.getPrimalSolution(x);
    u = Eigen::Map<JointVector>(x); // map back to eigen

    double obj_val = qp.getObjVal();
    if (isnan(obj_val) || isinf(obj_val)) {
        ROS_WARN_STREAM("MI QP: objective value = " << obj_val);
        u = JointVector::Zero();
    }

    return status;
}


void IKOptimizer::calc_objective(const Eigen::Affine3d& Td, const Vector6d& Vd,
                                  const JointVector& q, const JointVector& dq,
                                  double fd, const Eigen::Vector3d& force,
                                  const Eigen::Vector3d& torque,
                                  const Eigen::Vector3d& pc,
                                  double dt, JointMatrix& H, JointVector& g) {
    // Calculate mapping from base inputs to generalized coordinates. The
    // former are in the base frame while the latter are in the world frame.
    JointMatrix B = JointMatrix::Identity();
    Kinematics::calc_base_input_mapping(q, B);

    // Calculate pose error.
    Vector6d P_err;
    calc_pose_error(Td, q, P_err);

    // Calculate Jacobian.
    JacobianMatrix J;
    Kinematics::jacobian(q, J);
    JacobianMatrix JB = J * B;

    // If applied force is suitably large, then we update the contact direction
    // unit vector nf.
    double f_norm = force.norm();
    // if (f_norm > FORCE_THRESHOLD) {
    //     nf = f / f_norm;
    // }

    /*************************************************************************/
    /* 1. Minimize velocity objective. */

    JointMatrix Q1 = JointMatrix::Identity();
    JointVector C1 = JointVector::Zero();

    // To reduce base movement, increase the base joint weighting.
    // Q1.topLeftCorner<3, 3>() = 10 * Eigen::Matrix3d::Identity();

    /*************************************************************************/
    /* 2. Avoid joint limits objective. */

    // Base has no limits, so first three joints are unweighted.
    JointMatrix Q2 = dt * dt * JointMatrix::Identity();
    Q2.topLeftCorner<3, 3>() = Eigen::Matrix3d::Zero();

    JointVector C2 = dt * (2*q - (POSITION_LIMITS_UPPER + POSITION_LIMITS_LOWER));
    C2.head<3>() = Eigen::Vector3d::Zero();

    /*************************************************************************/
    /* 3. Error minimization objective */

    Matrix6d Kp = Matrix6d::Identity();
    // Kp.diagonal() << 1, 1, 1, 0, 0, 0;

    Matrix6d W3 = Matrix6d::Identity();
    // W3.diagonal() << 1, 1, 1, 0, 0, 0;

    JointMatrix Q3 = JB.transpose() * W3 * JB;
    JointVector C3 = -(Kp * P_err + Vd).transpose() * W3 * JB;

    /*************************************************************************/
    /* 4. Minimize joint acceleration */

    JointMatrix Q4 = JointMatrix::Zero();
    JointVector C4 = JointVector::Zero();

    // Only do numerical differentiation with a non-zero timestep.
    // if (dt * dt > 0) {
    //     Q4 = JointMatrix::Identity() / (dt * dt);
    //     C4 = -dq.transpose() * Q4;
    // }


    /*************************************************************************/
    /* 5. Force compliance */

    // Limit the maximum force applied by compliance. Not a feature of the
    // controller, but reasonable for safety when interacting with people.
    // TODO does this make sense? shouldn't this make it so that the robot
    // eventually stops deviating from its position?
    Eigen::Vector3d f_compliant = force;
    Eigen::Vector3d torque_compliant = torque;
    if (f_norm > MAX_COMPLIANCE_FORCE) {
        f_compliant = MAX_COMPLIANCE_FORCE * force / f_norm;
    }

    // Only start to comply with forces above a certain magnitude, to reject
    // noise.
    // if (f_norm > FORCE_THRESHOLD) {
    //     double alpha_compliant = 2;
    //     double b_compliant = 1 - f_norm / FORCE_THRESHOLD;  // TODO: bad name
    //     f_compliant = f * (1 - exp(alpha_compliant * b_compliant));
    // } else {
    //     f_compliant = Eigen::Vector3d::Zero();
    // }
    Vector6d wrench_compliant;
    wrench_compliant << f_compliant, torque_compliant;

    // ROS_INFO_STREAM("force = " << force);
    // ROS_INFO_STREAM("torque = " << torque);

    Matrix6d Kp_com = Matrix6d::Zero();
    Kp_com.diagonal() << 1, 1, 0, 1, 1, 1;

    // only comply in directions where gain is non-zero
    // 0.01 is pretty good for human interaction; could even probably go a bit
    // higher
    Matrix6d Kf_com = Matrix6d::Zero();
    // Matrix6d Kf_com = 0.01 * Matrix6d::Identity();
    Kf_com.diagonal() << 0, 0, 0.005, 0, 0, 0;
    // Kf_com.diagonal() << 0, 0, 0, 0.01, 0.01, 0.01;

    // TODO should take in fd as a vector
    Vector6d wrench_compliant_d = Vector6d::Zero();
    // wrench_compliant_d << 0, 0, fd, 0, 0, 0;
    wrench_compliant_d << 0, 0, -10, 0, 0, 0;
    Vector6d W_err = wrench_compliant_d - wrench_compliant;
    // ROS_INFO_STREAM("Wd = " << wrench_compliant_d);

    Vector6d Vc = Vd + Kp_com * P_err + Kf_com * W_err;
    // ROS_INFO_STREAM("Vc = " << Vc);

    Matrix6d W5 = Matrix6d::Identity();
    JointMatrix Q5 = JB.transpose() * W5 * JB;
    JointVector C5 = -Vc.transpose() * W5 * JB;

    /*************************************************************************/
    /* 6. Orientation of EE tracks nf. */
    // TODO delete: unused

    // Eigen::Matrix3d Re = w_T_tool.rotation();
    // Eigen::Vector3d ae = Re.col(2);
    //
    // Matrix3x9 Jn, Js, Ja;
    // calc_rotation_error_jacobians(q, Jn, Js, Ja);
    //
    // Eigen::Matrix3d W6 = Eigen::Matrix3d::Identity();
    // JointMatrix Q6 = dt * dt * Ja.transpose() * W6 * Ja;
    // JointVector C6 = dt * (ae - nf).transpose() * W6 * Ja;
    JointMatrix Q6 = JointMatrix::Zero();
    JointVector C6 = JointVector::Zero();

    /*************************************************************************/
    /* 7. Pushing */

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
        // ROS_INFO_STREAM("f_xy = " << f_xy);
    } else {
        double alpha_nf = 0.01;
        nf_xy = slerp2d(nf_xy, np_xy, alpha_nf);
    }

    // push direction balances between the two
    double alpha_push = 0.5;
    Eigen::Vector2d push_dir = slerp2d(nf_xy, np_xy, -alpha_push);

    // TODO fixed gains for now
    Vector6d Vp = Vector6d::Zero();
    Vp << 0.2 * push_dir, 1 * P_err.tail<4>();

    // ROS_INFO_STREAM("np_xy = " << np_xy);
    // ROS_INFO_STREAM("nf_xy = " << nf_xy);
    // ROS_INFO_STREAM("push_dir = " << push_dir);

    // Matrix3x9 Jp = J.topRows<3>();
    Matrix6d W7 = Matrix6d::Identity();
    JointMatrix Q7 = B.transpose() * J.transpose() * W7 * J * B;
    JointVector C7 = -Vp.transpose() * W7 * J * B;

    // for safety, in case end of trajectory is reached
    // if (P_err.head<2>().norm() <= 0.2) {
    //     ROS_INFO_STREAM("near end, stopping");
    //     Q7 = JointMatrix::Zero();
    //     C7 = JointVector::Zero();
    // }

    // get rid of these for now to avoid accidental mess ups
    Q7 = JointMatrix::Zero();
    C7 = JointVector::Zero();

    /*************************************************************************/
    /* Manipulability */

    // JointVector m_grad;
    // Kinematics::manipulability_gradient_analytic(q, m_grad);
    // JointVector C_mi = -dt * B.transpose() * m_grad;

    /*************************************************************************/
    /* Objective weighting */

    double w1 = 1.0; // minimize velocity -- this should typically be 1.0
    double w2 = 0.0; // avoid joint limits
    double w3 = 100.0; // minimize pose error
    double w4 = 0.0; // minimize acceleration
    double w5 = 0.0; // force compliance
    double w6 = 0.0; // force-based orientation
    double w7 = 0.0; // pushing
    double w_mi = 0.0;

    if (freaked_out && !did_print_gs) {
        ROS_WARN_STREAM("C1 = " << C1);
        ROS_WARN_STREAM("C2 = " << C2);
        ROS_WARN_STREAM("C3 = " << C3);
        ROS_WARN_STREAM("C4 = " << C4);
        ROS_WARN_STREAM("C5 = " << C5);
        ROS_WARN_STREAM("C6 = " << C6);
        ROS_WARN_STREAM("C7 = " << C7);
        did_print_gs = true;
    }

    H = w1*Q1 + w2*Q2 + w3*Q3 + w4*Q4 + w5*Q5 + w6*Q6 + w7*Q7;
    g = w1*C1 + w2*C2 + w3*C3 + w4*C4 + w5*C5 + w6*C6 + w7*C7; // + w_mi*C_mi;
}


int IKOptimizer::solve(double t, PoseTrajectory& trajectory,
                       const JointVector& q, const JointVector& dq,
                       double fd, const Eigen::Vector3d& force,
                       const Eigen::Vector3d& torque,
                       const Eigen::Vector3d& pc,
                       const std::vector<ObstacleModel>& obstacles, double dt,
                       JointVector& u) {
    Eigen::Affine3d Td;
    Vector6d Vd;
    trajectory.sample(t, Td, Vd);

    /*** OBJECTIVE ***/

    JointMatrix H;
    JointVector g;
    calc_objective(Td, Vd, q, dq, fd, force, torque, pc, dt, H, g);


    /*** BOUNDS ***/

    // qpOASES makes the distinction between bounds and constraints.
    // Bounds are of the form lb <= x <= ub, whereas constraints are an
    // affine transformation of the optimization variable, lb <= Ax <=
    // ub

    // Velocity damper inequality constraints
    JointVector dq_lb, dq_ub;
    velocity_damper_limits(q, dq_lb, dq_ub);

    // Base FoV constraint -- in the simplest case, it just constrains the
    // base not to move backward.
    //dq_lb(0) = std::max(dq_lb(0), 0.0);


    /*** CONSTRAINTS ***/

    // Obstacle constraints.
    Eigen::MatrixXd A_obs;
    Eigen::VectorXd ub_obs;
    Eigen::Vector2d pb(q[0], q[1]);
    calc_obstacle_limits(pb, obstacles, A_obs, ub_obs);


    /*** SOLVE QP ***/

    JointVector u1, u2;
    int status = solve_qp(H, g, A_obs, dq_lb, dq_ub, ub_obs, u1);
    u = u1;

    // // MI as a secondary objective, solved in the null space
    // JacobianMatrix J;
    // JointMatrix B;
    // Kinematics::jacobian(q, J);
    // Kinematics::calc_base_input_mapping(q, B);
    //
    // JacobianMatrix A_eq = J * B;
    // Vector6d b_eq = A_eq * u1;
    //
    // JointVector m_grad;
    // Kinematics::manipulability_gradient_analytic(q, m_grad);
    // double w_mi = 0;
    // JointVector g_mi = -w_mi * dt * B.transpose() * m_grad;
    //
    // int status_mi_qp = solve_qp_mi(g_mi, dq_lb, dq_ub, A_eq, b_eq, A_obs, ub_obs, u2);
    // if (status_mi_qp) {
    //     ROS_WARN_STREAM("MI QP infeasible: status = " << status_mi_qp);
    //     u2 = JointVector::Zero();
    // }
    // u = u2;

    return status;
}


void IKOptimizer::velocity_damper_limits(const JointVector& q, JointVector& dq_lb,
                                         JointVector& dq_ub) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
        // Upper bound
        double del_q_ub = POSITION_LIMITS_UPPER(i) - q(i);
        dq_ub(i) = VELOCITY_LIMITS_UPPER(i);

        if (POSITIION_LIMITED[i] && del_q_ub <= INFLUENCE_DIST(i)) {
            ROS_WARN_STREAM("Joint " << JOINT_NAMES[i]
                    << " within influence distance of upper bound.");
            dq_ub(i) *= (del_q_ub - SAFETY_DIST(i))
                      / (INFLUENCE_DIST(i) - SAFETY_DIST(i));
        }

        // Lower bound
        double del_q_lb = q(i) - POSITION_LIMITS_LOWER(i);
        dq_lb(i) = VELOCITY_LIMITS_LOWER(i);

        if (POSITIION_LIMITED[i] && del_q_lb <= INFLUENCE_DIST(i)) {
            ROS_WARN_STREAM("Joint " << JOINT_NAMES[i]
                    << " within influence distance of lower bound.");
            dq_lb(i) *= (del_q_lb - SAFETY_DIST(i))
                      / (INFLUENCE_DIST(i) - SAFETY_DIST(i));
        }
    }
}


int IKOptimizer::calc_obstacle_limits(const Eigen::Vector2d& pb,
                                 const std::vector<ObstacleModel>& obstacles,
                                 Eigen::MatrixXd& A, Eigen::VectorXd& b) {
    // Only consider obstacles within the influence distance.
    std::vector<ObstacleModel> close_obstacles;
    filter_obstacles(pb, obstacles, close_obstacles);

    // for (int i = 0; i < close_obstacles.size(); ++i) {
    //     ROS_INFO_STREAM("obs[" << i << "] = " << close_obstacles[i].centre());
    // }
    // return 0;

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

        double limit = OBS_COEFF * (d - OBS_SAFETY_DIST)
            / (OBS_INFLUENCE_DIST - OBS_SAFETY_DIST);

        A(i,0) = n(0);
        A(i,1) = n(1);
        b(i)   = limit;
    }

    return num_obs;
}


void IKOptimizer::filter_obstacles(const Eigen::Vector2d& pb,
                                   const std::vector<ObstacleModel>& obstacles,
                                   std::vector<ObstacleModel>& close_obstacles) {
    for (int i = 0; i < obstacles.size(); ++i) {
        if (obstacles[i].in_range(pb)) {
            close_obstacles.push_back(obstacles[i]);
        }
    }
}


void IKOptimizer::get_state(IKOptimizerState& state) {
    state = this->state;
}

} // namespace mm

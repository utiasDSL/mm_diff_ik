#include "mm_motion_control/pose_control/optimizer.h"

#include <Eigen/Eigen>
#include <qpOASES/qpOASES.hpp>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/differentiation.h>

#include "mm_motion_control/pose_control/obstacle.h"
#include "mm_motion_control/pose_control/pose_error.h"
#include "mm_motion_control/pose_control/rate.h"


namespace mm {


static const int NUM_WSR = 50;


bool IKOptimizer::init() {
    nf << 1, 0, 0;
    nf_xy << 1, 0;
    return true;
}

int IKOptimizer::solve_qp(JointMatrix& H, JointVector& g, Eigen::MatrixXd& A,
                          JointVector& lb, JointVector& ub,
                          Eigen::VectorXd& ubA, JointVector& dq_opt) {
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
    qp.setPrintLevel(qpOASES::PL_NONE);
    qpOASES::returnValue ret = qp.init(H_data, g_data, A_data, lb_data, ub_data,
                                       lbA_data, ubA_data, nWSR);
    int status = qpOASES::getSimpleStatus(ret);

    qpOASES::real_t dq_opt_raw[NUM_JOINTS];
    qp.getPrimalSolution(dq_opt_raw);
    dq_opt = Eigen::Map<JointVector>(dq_opt_raw); // map back to eigen

    // Populate state values;
    state.dq_opt = dq_opt;
    state.obj_val = qp.getObjVal();
    state.code = ret;
    state.status = status;

    return status;
}


void IKOptimizer::calc_objective(const Eigen::Affine3d& Td, const Vector6d& Vd,
                                  const JointVector& q, const JointVector& dq,
                                  double fd, const Eigen::Vector3d& f,
                                  const Eigen::Vector3d& pc,
                                  double dt, JointMatrix& H, JointVector& g) {
    // Calculate mapping from base inputs to generalized coordinates. The
    // former are in the base frame while the latter are in the world frame.
    JointMatrix B = JointMatrix::Identity();
    Kinematics::calc_base_input_mapping(q, B);

    // If applied force is suitably large, then we update the contact direction
    // unit vector nf.
    double f_norm = f.norm();
    // if (f_norm > FORCE_THRESHOLD) {
    //     nf = f / f_norm;
    // }

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

    JacobianMatrix dPe_dq;
    Vector6d Pe;
    calc_pose_error(Td, q, Pe);
    calc_pose_error_jacobian(Td, q, dPe_dq);

    JacobianMatrix dPe_dqB = dPe_dq * B;

    Matrix6d Kp = Matrix6d::Identity();

    Matrix6d W3 = Matrix6d::Zero();
    W3.diagonal() << 0, 0, 0, 1.0, 1.0, 1.0;

    // NOTE: here we're relying on the fact that J4 is negative, which is
    // actually quite confusing
    JointMatrix Q3 = dPe_dqB.transpose() * W3 * dPe_dqB;
    JointVector C3 = (Kp * Pe + Vd).transpose() * W3 * dPe_dqB;

    /*************************************************************************/
    /* 4. Minimize joint acceleration */

    // Only do numerical differentiation with a non-zero timestep.
    JointMatrix Q4 = JointMatrix::Zero();
    if (dt * dt > 0) {
        Q4 = JointMatrix::Identity() / (dt * dt);
    }
    JointVector C4 = -dq.transpose() * Q4;


    /*************************************************************************/
    /* 5. Force compliance */

    // Limit the maximum force applied by compliance. Not a feature of the
    // controller, but reasonable for safety when interacting with people.
    // TODO does this make sense? shouldn't this make it so that the robot
    // eventually stops deviating from its position?
    Eigen::Vector3d f_compliant = f;
    if (f_norm > MAX_COMPLIANCE_FORCE) {
        f_compliant = MAX_COMPLIANCE_FORCE * f / f_norm;
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
    f_compliant = f;
    // ROS_INFO_STREAM("fc = " << f_compliant);

    Eigen::Matrix3d Kp_com = Eigen::Matrix3d::Zero();
    Kp_com.diagonal() << 0, 1.0, 1.0;
    // Eigen::Matrix3d Kf_com = 0.005 * Eigen::Matrix3d::Identity();

    // only comply in certain directions
    Eigen::Matrix3d Kf_com = Eigen::Matrix3d::Zero();
    Kf_com.diagonal() << 0.005, 0, 0;

    // TODO duplication of effort between here and pose objective
    Eigen::Affine3d w_T_tool;
    Kinematics::calc_w_T_tool(q, w_T_tool);
    Eigen::Vector3d pe = w_T_tool.translation();  // TODO bad name
    Eigen::Vector3d pd = Td.translation();

    JacobianMatrix J;
    Kinematics::jacobian(q, J);
    Matrix3x9 Jp = J.topRows<3>();

    // TODO kludge for now
    Eigen::Vector3d f_compliant_d;
    // f_compliant_d << fd, 0, 0;
    f_compliant_d << 5, 0, 0;

    // Matrix3x9 Af = -(Bf + dt * Kf) * Jp;
    // Eigen::Vector3d df = Kf * (pd - pe) - f_compliant;
    // Eigen::Vector3d df_og = (Kf_og * (pd - pe) - f_compliant) / dt;
    Matrix3x9 Af = Jp * B;
    Eigen::Vector3d df = Vd.head<3>() + Kp_com * (pd - pe) + Kf_com * (f_compliant_d - f_compliant);

    // ROS_INFO_STREAM("vc = " << df);

    Eigen::Matrix3d W5 = Eigen::Matrix3d::Identity();
    JointMatrix Q5 = Af.transpose() * W5 * Af;
    JointVector C5 = -df.transpose() * W5 * Af;

    /*************************************************************************/
    /* 6. Orientation of EE tracks nf. */

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
    /* 7. TODO: Pushing */
    Eigen::Vector3d p_err = pd - pe;

    // position normal
    double p_err_norm = p_err.head<2>().norm();
    Eigen::Vector2d np_xy = Eigen::Vector2d::Zero();
    if (p_err_norm > 1e-2) {
        np_xy = p_err.head<2>() / p_err_norm;
    }

    // nf_xy decays back to the push direction in the absence of sufficiently
    // large force measurements (i.e. ones that aren't noise)
    Eigen::Vector2d f_xy = f.head<2>();
    double f_xy_norm = f_xy.norm();
    if (f_xy_norm > FORCE_THRESHOLD) {
        nf_xy = f_xy / f_xy_norm;
    } else {
        double alpha_nf = 0.9;
        // TODO this is no longer a unit vector, necessarily---does that matter?
        nf_xy = alpha_nf * nf_xy + (1 - alpha_nf) * np_xy;
        if (nf_xy.norm() > 1e-3) {
            nf_xy = nf_xy / nf_xy.norm();
        }
    }

    // force normal
    // Eigen::Vector2d nf_xy = nf.head<2>();

    // push direction balances between the two
    double alpha_push = 0.5;
    Eigen::Vector2d push_dir = ((1+alpha_push)*nf_xy*nf_xy.transpose() - alpha_push*Eigen::Matrix2d::Identity()) * np_xy;

    // TODO fixed gains for now
    double push_dir_norm = push_dir.norm();
    Eigen::Vector3d vp = Eigen::Vector3d::Zero();
    vp(2) = 1 * p_err(2); // z
    if (push_dir_norm > 1e-2) {
        vp.head<2>() << 0.2 * push_dir.head<2>() / push_dir_norm;
    }

    // ROS_INFO_STREAM("vp = " << vp);

    Eigen::Matrix3d W7 = Eigen::Matrix3d::Identity();
    JointMatrix Q7 = B.transpose() * Jp.transpose() * W7 * Jp * B;
    JointVector C7 = -vp.transpose() * W7 * Jp * B;

    /*************************************************************************/
    /* Objective weighting */

    double w1 = 1.0; // minimize velocity -- this should typically be 1.0
    double w2 = 0.0; // avoid joint limits
    double w3 = 0.0; // minimize pose error
    double w4 = 0.0; // minimize acceleration
    double w5 = 0.0; // force compliance
    double w6 = 0.0; // force-based orientation
    double w7 = 1.0; // pushing

    H = w1*Q1 + w2*Q2 + w3*Q3 + w4*Q4 + w5*Q5 + w6*Q6 + w7*Q7;
    g = w1*C1 + w2*C2 + w3*C3 + w4*C4 + w5*C5 + w6*C6 + w7*C7;
}


int IKOptimizer::solve(double t, PoseTrajectory& trajectory,
                       const JointVector& q, const JointVector& dq,
                       double fd, const Eigen::Vector3d& f,
                       const Eigen::Vector3d& pc,
                       const std::vector<ObstacleModel>& obstacles, double dt,
                       JointVector& dq_opt) {
    // TODO is this slow? Could potentially be improved by pre-processing the
    // trajectory
    // Look one timestep into the future to see where we want to end up.
    Eigen::Affine3d Td;
    Vector6d Vd;
    // trajectory.sample(t + CONTROL_TIMESTEP, Td, Vd);
    trajectory.sample(t, Td, Vd);

    /*** OBJECTIVE ***/

    JointMatrix H;
    JointVector g;
    calc_objective(Td, Vd, q, dq, fd, f, pc, dt, H, g);


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
    Eigen::VectorXd b_obs;
    Eigen::Vector2d pb(q[0], q[1]);
    calc_obstacle_limits(pb, obstacles, A_obs, b_obs);


    /*** SOLVE QP ***/

    int status = solve_qp(H, g, A_obs, dq_lb, dq_ub, b_obs, dq_opt);


    /*** CALCULATE OBJECTIVE FUNCTION VALUES ***/

    // // Examine objective function values
    // double vel_obj = dq_opt.transpose() * Q1 * dq_opt; // want to minimize
    //
    // double mi = Kinematics::manipulability(q);
    // double mi_obj_quad = dt * dt * dq_opt.transpose() * Hm * dq_opt;
    // double mi_obj_lin = dt * dm.dot(dq_opt);
    //
    // // This is what actually gets considering in the optimization
    // // problem.
    // double mi_obj = w1 * (mi_obj_quad + mi_obj_lin); // want to maximize
    //
    // publish_state(dq_opt, vel_obj, mi_obj);

    return status;
}


void IKOptimizer::linearize_manipulability1(const JointVector& q,
                                            JointVector& dm, double h) {
    dm = JointVector::Zero();
    JointMatrix I = JointMatrix::Identity();

    // MI does not change as w.r.t. the base joint variables, so we leave the
    // first three joints as zero.
    for (int i = 3; i < NUM_JOINTS; ++i) {
        JointVector Ii = I.col(i);
        double m1 = Kinematics::manipulability(q + h*Ii);
        double m2 = Kinematics::manipulability(q - h*Ii);
        dm(i) = (m1 - m2) / (2*h);
    }
}


void IKOptimizer::linearize_manipulability2(const JointVector& q,
                                            JointVector& dm, JointMatrix& Hm,
                                            double h) {
    approx_gradient<NUM_JOINTS>(&Kinematics::manipulability, h, q, dm);
    approx_hessian<NUM_JOINTS>(&Kinematics::manipulability, h, q, Hm);
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

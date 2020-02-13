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


void IKOptimizer::build_objective(const Eigen::Affine3d& Td, const Vector6d& twistd,
                                  const JointVector& q, const JointVector& dq,
                                  double fd, const Eigen::Vector3d& f,
                                  double dt, JointMatrix& H, JointVector& g) {
    /* 1. Minimize velocity objective. */

    JointMatrix Q1 = JointMatrix::Identity();
    JointVector C1 = JointVector::Zero();

    // To reduce base movement, increase the base joint weighting.
    // Q1.topLeftCorner<3, 3>() = 10 * Eigen::Matrix3d::Identity();

    /* 2. Manipulability objective. */

    JointVector dm = JointVector::Zero();
    JointMatrix Hm = JointMatrix::Zero();
    // linearize_manipulability1(q, dm, STEP_SIZE);
    // linearize_manipulability2(q, dm, Hm, STEP_SIZE);

    JointMatrix Q2 = -dt * dt * Hm;
    JointVector C2 = -dt * dm;

    /* 3. Avoid joint limits objective. */

    // Base has no limits, so first three joints are unweighted.
    JointMatrix Q3 = dt * dt * JointMatrix::Identity();
    Q3.topLeftCorner<3, 3>() = Eigen::Matrix3d::Zero();

    JointVector C3 = dt * (2*q - (POSITION_LIMITS_UPPER + POSITION_LIMITS_LOWER));
    C3.head<3>() = Eigen::Vector3d::Zero();

    /* 4. Error minimization objective */

    JacobianMatrix J4;
    Vector6d e4;
    pose_error(Td, q, e4);
    pose_error_jacobian(Td, q, J4);

    Matrix6d Kp = Matrix6d::Identity();
    Matrix6d W4 = Matrix6d::Identity();

    // We may choose to weight orientation error differently than position.
    W4.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
    W4.bottomRightCorner<3, 3>() = 0.0*Eigen::Matrix3d::Identity();

    // Use feedforward to improve tracking performance without requiring huge
    // gains.
    // TODO here we're relying on the fact that J4 is negative, which is
    // actually quite confusing
    JointMatrix Q4 = J4.transpose() * W4 * J4;
    JointVector C4 = (Kp * e4 + twistd).transpose() * W4 * J4;

    /* 5. Minimize joint acceleration */

    // Only do numerical differentiation with a non-zero timestep.
    JointMatrix Q5 = JointMatrix::Zero();
    if (dt * dt > 0) {
        Q5 = JointMatrix::Identity() / (dt * dt);
    }
    JointVector C5 = -dq.transpose() * Q5;

    /* 6. Force compliance */

    // Limit the maximum force applied by compliance. Not a feature of the
    // controller, but reasonable for safety when interacting with people.
    double f_norm = f.norm();
    Eigen::Vector3d f_compliant = f;
    if (f_norm > MAX_COMPLIANCE_FORCE) {
        f_compliant = MAX_COMPLIANCE_FORCE * f / f_norm;
    }

    // Only start to comply with forces above a certain magnitude, to reject
    // noise.
    if (f_norm > FORCE_THRESHOLD) {
        f_compliant = f * (1 - FORCE_THRESHOLD / f_norm);
    } else {
        f_compliant = Eigen::Vector3d::Zero();
    }

    Eigen::Matrix3d Kf = 0.5 * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Bf = 0.0 * Eigen::Matrix3d::Identity();

    Eigen::Affine3d w_T_tool;
    Kinematics::calc_w_T_tool(q, w_T_tool);
    Eigen::Vector3d pe = w_T_tool.translation();
    Eigen::Vector3d pd = Td.translation();

    JacobianMatrix J;
    Kinematics::jacobian(q, J);
    Matrix3x9 Jp = J.topRows<3>();

    // TODO we can include feedforward velocity here
    Matrix3x9 Af = -(Bf + dt * Kf) * Jp;
    Eigen::Vector3d df = Kf * (pd - pe) - f_compliant;

    Eigen::Matrix3d W6 = Eigen::Matrix3d::Identity();
    JointMatrix Q6 = Af.transpose() * W6 * Af;
    JointVector C6 = df.transpose() * W6 * Af;

    /* 7. Force regulation */

    JointMatrix Q7 = JointMatrix::Zero();
    JointVector C7 = JointVector::Zero();
    double kp = 0.001;

    // If applied force is suitably large, then we update the contact direction
    // unit vector nf.
    if (f_norm > FORCE_THRESHOLD) {
        nf = f / f_norm;
    }

    // For now, only do regulation for > 0 desired forces. TODO generalize to 0
    // and negative values.
    if (fd > 0) {
        double f_err = fd - f_norm;
        Q7 = Jp.transpose() * nf * nf.transpose() * Jp;
        C7 = -kp * f_err * nf.transpose() * Jp;
        ROS_INFO_STREAM(f_err);
    }

    /* 8. Orientation of EE tracks nf. */

    Eigen::Matrix3d Re = w_T_tool.rotation();
    Eigen::Vector3d ae = Re.col(2);

    Matrix3x9 Jn, Js, Ja;
    rotation_error_jacobians(q, Jn, Js, Ja);

    Eigen::Matrix3d W8 = Eigen::Matrix3d::Identity();
    JointMatrix Q8 = dt * dt * Ja.transpose() * W8 * Ja;
    JointVector C8 = dt * (ae - nf).transpose() * W8 * Ja;

    /* 9. Tangent trajectory tracking. */

    // TODO in the case of Task 1, instead of enforcing a position trajectory,
    // we could actually enforce velocity in the tangent directions should be
    // zero. This may work better.

    // We use pd(1:2) as our 2D trajectory, and project pe into the tangent
    // (null) space.
    Eigen::Vector2d pd2 = pd.tail<2>();
    Eigen::Vector2d vd2 = twistd.segment<2>(1);
    Eigen::FullPivLU<Eigen::Matrix<double, 1, 3>> lu(nf.transpose());
    Eigen::MatrixXd V = lu.kernel();

    // TODO if we put d9 = 0, this enforces a zero velocity constraint in the
    // tangent plane.
    Eigen::Vector2d d9 = -(pd2 - V.transpose()*pe) - vd2;
    Eigen::Matrix<double, 2, 9> J9 = V.transpose()*Jp;

    Eigen::Matrix2d W9 = Eigen::Matrix2d::Identity();
    JointMatrix Q9 = J9.transpose() * W9 * J9;
    JointVector C9 = d9.transpose() * W9 * J9;

    /* 10. Push object */

    // Desired velocity vector.
    // TODO we could actually make this 3D to punish velocity in the other
    // directions. -- except the other directions should be used to return to
    // the correct force orientation.
    // TODO we can try this with and without the force orientation stuff
    Eigen::Vector3d vd;
    vd << 0.1, 0, 0;
    double vd_norm = vd.norm();
    Eigen::Vector3d vd_unit = vd / vd_norm;

    // Track desired EE velocity along a single direction.
    JointMatrix Q10 = Jp.transpose() * vd_unit * vd_unit.transpose() * Jp;
    JointVector C10 = -vd.transpose() * Jp;

    Eigen::Matrix3d Kv = 0.1*Eigen::Matrix3d::Identity();

    // Make nf track direction of velocity.
    Eigen::Matrix3d W11 = Eigen::Matrix3d::Identity();
    JointMatrix Q11 = Jp.transpose() * W11 * Jp;
    JointVector C11 = (Kv * (vd_unit - nf)).transpose() * W11 * Jp;


    /* Objective weighting */

    double w1 = 1.0; // velocity
    double w2 = 0.0; // manipulability
    double w3 = 0.0; // joint limits
    double w4 = 100.0; // pose error
    double w5 = 1.0; // acceleration
    double w6 = 0.0; // force compliance
    double w7 = 0.0; // force regulation
    double w8 = 0.0; // orientation tracks nf
    double w9 = 0.0; // 2d position error
    double w10 = 0.0; // track velocity line
    double w11 = 0.0; // nf tracks velocity direction

    H = w1*Q1 + w2*Q2 + w3*Q3 + w4*Q4 + w5*Q5 + w6*Q6 + w7*Q7 + w8*Q8 + w9*Q9 + w10*Q10 + w11*Q11;
    g = w1*C1 + w2*C2 + w3*C3 + w4*C4 + w5*C5 + w6*C6 + w7*C7 + w8*C8 + w9*C9 + w10*C10 + w11*C11;
}


int IKOptimizer::solve(double t, PoseTrajectory& trajectory,
                       const JointVector& q, const JointVector& dq,
                       double fd, const Eigen::Vector3d& f,
                       const std::vector<ObstacleModel>& obstacles, double dt,
                       JointVector& dq_opt) {
    // TODO is this slow? Could potentially be improved by pre-processing the
    // trajectory
    // Look one timestep into the future to see where we want to end up.
    Eigen::Affine3d Td;
    Vector6d twistd;
    // trajectory.sample(t + CONTROL_TIMESTEP, Td, twistd);
    trajectory.sample(t, Td, twistd);

    /*** OBJECTIVE ***/

    JointMatrix H;
    JointVector g;
    build_objective(Td, twistd, q, dq, fd, f, dt, H, g);


    /*** BOUNDS ***/

    // qpOASES makes the distinction between bounds and constraints.
    // Bounds are of the form lb <= x <= ub, whereas constraints are an
    // affine transformation of the optimization variable, lb <= Ax <=
    // ub

    // Velocity damper inequality constraints
    JointVector dq_lb, dq_ub;
    velocity_damper_limits(q, dq_lb, dq_ub);


    /*** CONSTRAINTS ***/

    // Obstacle constraints.
    Eigen::MatrixXd A_obs;
    Eigen::VectorXd b_obs;
    Eigen::Vector2d pb(q[0], q[1]);
    obstacle_limits(pb, obstacles, A_obs, b_obs);


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


int IKOptimizer::obstacle_limits(const Eigen::Vector2d& pb,
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

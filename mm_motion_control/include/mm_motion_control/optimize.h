#pragma once

#include <Eigen/Eigen>
#include <QuadProg.h>
#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/differentiation.h>


// eigen-quadprog API:
//
// QuadProgDense(int nrvar, int nreq, int nrineq);
//
// bool solve(const MatrixXd& Q, const VectorXd& C,
// 	const MatrixXd& Aeq, const VectorXd& Beq,
// 	const MatrixXd& Aineq, const VectorXd& Bineq,
// 	bool isDecomp=false);

namespace mm {

// Distances for the velocity damper constraints.
static const double M_PI_6 = M_PI / 6.0;
static const JointVector INFLUENCE_DIST(
        (JointVector() << 0.5, 0.5, M_PI_6, M_PI_6, M_PI_6, M_PI_6, M_PI_6, M_PI_6, M_PI_6).finished());
static const JointVector SAFETY_DIST = 0.2 * INFLUENCE_DIST;

// Make the joint true if a position limit should be enforced, false otherwise.
static const bool POSITIION_LIMITED[] = {
    false, false, false,               /* base */
    true, true, true, true, true, true /* arm  */};


// For numerical differentiation
// NOTE pushing this down to even 1e-5 makes the numerical Hessian fail
static const double STEP_SIZE = 1e-3;


class IKOptimizer {
    public:
        IKOptimizer() {};

        // Create and solve the QP.
        //
        // q:      Current value of joint angles.
        // ee_vel: Desired task space velocity of end effector.
        // dt:     Control timestep.
        // dq_opt: Optimal values of joint velocities.
        //
        // Returns true if optimization problem was solved successfully
        // (i.e. constraints satisified).
        bool solve(const JointVector& q, const Vector6d& ee_vel, double dt,
                   JointVector& dq_opt) {
            /* OBJECTIVE */

            // Minimize velocity objective.
            JointMatrix W = JointMatrix::Identity(); // norm weighting

            // Manipulability objective.
            JointVector dm;
            JointMatrix Hm = JointMatrix::Zero();
            linearize_manipulability2(q, dm, Hm);
            // linearize_manipulability1(q, dm);

            double alpha = 0;//1000.0; // weighting of manipulability objective

            JointMatrix Q = W - alpha * dt * dt * Hm;
            JointVector C = -alpha * dt * dm;

            // JointMatrix Q = W;
            // JointVector C = JointVector::Zero();

            /* EQUALITY CONSTRAINTS */

            // Track the reference velocity.
            // TODO relaxation
            JacobianMatrix J;
            Kinematics::jacobian(q, J);

            Eigen::Matrix<double, 6, 9> Aeq = J;
            Eigen::Matrix<double, 6, 1> Beq = ee_vel;

            /* INEQUALITY CONSTRAINTS */

            Eigen::Matrix<double, 1, 0> Aineq, Bineq;

            // Velocity damper ineq constraints
            // JointVector dq_lb, dq_ub;
            // velocity_damper_limits(q, dq_lb, dq_ub);
            //
            // Eigen::Matrix<double, 2*NUM_JOINTS, NUM_JOINTS> Aineq;
            // Aineq << -JointMatrix::Identity(), JointMatrix::Identity();
            //
            // Eigen::Matrix<double, 2*NUM_JOINTS, 1> Bineq;
            // Bineq << -dq_lb, dq_ub;

            // Solve the QP.
            // Arguments: # variables, # eq constraints, # ineq constraints
            Eigen::QuadProgDense qp(NUM_JOINTS, 6, 0/*2*NUM_JOINTS*/);
            bool success = qp.solve(Q, C, Aeq, Beq, Aineq, Bineq);
            dq_opt = qp.result();

            // Examine objective function values
            double vel_obj = dq_opt.transpose() * W * dq_opt; // want to minimize

            double mi = Kinematics::manipulability(q);
            double mi_obj_quad = alpha * dt * dt * dq_opt.transpose() * Hm * dq_opt;
            double mi_obj_lin = alpha * dt * dm.dot(dq_opt);
            double mi_obj = mi + mi_obj_quad + mi_obj_lin; // want to maximize

            // Compare to MI of numerically-integrated next position.
            double mi_next = Kinematics::manipulability(q + dt*dq_opt);

            // ROS_INFO_STREAM("vel obj = " << vel_obj << " mi obj = " << mi_obj << " mi = " << mi);

            return success;
        }

        // 1st-order linearization of manipulability index around joint values
        // q, returning the gradient dm.
        void linearize_manipulability1(const JointVector& q, JointVector& dm,
                                       double h=STEP_SIZE) {
            approx_gradient<NUM_JOINTS>(&Kinematics::manipulability, h, q, dm);
        }

        // 2nd-order linearization of manipulability index around joint values
        // q, returning the gradient dm, and Hessian Hm.
        void linearize_manipulability2(const JointVector& q, JointVector& dm,
                                       JointMatrix& Hm, double h=STEP_SIZE) {
            approx_gradient<NUM_JOINTS>(&Kinematics::manipulability, h, q, dm);
            approx_hessian<NUM_JOINTS>(&Kinematics::manipulability, h, q, Hm);
        }

        // Compute velocity constraints, including damping as the position
        // limits are approached.
        void velocity_damper_limits(const JointVector& q, JointVector& dq_lb,
                                    JointVector& dq_ub) {
            for (int i = 0; i < NUM_JOINTS; ++i) {
                double del_q_ub = POSITION_LIMITS_UPPER(i) - q(i);
                dq_ub(i) = VELOCITY_LIMITS_UPPER(i);
                if (POSITIION_LIMITED[i] && del_q_ub <= INFLUENCE_DIST(i)) {
                    dq_ub(i) *= (del_q_ub - SAFETY_DIST(i))
                              / (INFLUENCE_DIST(i) - SAFETY_DIST(i));
                }

                double del_q_lb = q(i) - POSITION_LIMITS_LOWER(i);
                dq_lb(i) = VELOCITY_LIMITS_LOWER(i);
                if (POSITIION_LIMITED[i] && del_q_lb <= INFLUENCE_DIST(i)) {
                    dq_lb(i) *= (del_q_lb - SAFETY_DIST(i))
                              / (INFLUENCE_DIST(i) - SAFETY_DIST(i));
                }
            }
        }
}; // class IKOptimizer

} // namespace mm

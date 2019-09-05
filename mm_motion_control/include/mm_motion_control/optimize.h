#pragma once

#include <Eigen/Eigen>
#include <QuadProg.h>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <mm_msgs/OptimizationState.h>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/differentiation.h>

#include "mm_motion_control/obstacle.h"


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
static const JointVector INFLUENCE_DIST((JointVector() <<
    0.5, 0.5, M_PI_6,                               /* base */
    M_PI_4, M_PI_4, M_PI_4, M_PI_4, M_PI_4, M_PI_4  /* arm */
).finished());

static const JointVector SAFETY_DIST = 0.25 * INFLUENCE_DIST;

// Make the joint true if a position limit should be enforced, false otherwise.
static const bool POSITIION_LIMITED[] = {
    false, false, false,               /* base */
    true, true, true, true, true, true /* arm  */
};


// For numerical differentiation
// NOTE pushing this down to even 1e-5 makes the numerical Hessian fail
static const double STEP_SIZE = 1e-3;

static const double OBS_SAFETY_DIST = 0.2;
static const double OBS_INFLUENCE_DIST = 0.4;
static const double OBS_COEFF = 1.0;


class IKOptimizer {
    public:
        IKOptimizer() {};

        bool init(ros::NodeHandle& nh) {
            state_pub.reset(new StatePublisher(nh, "/optimization_state", 1));
        }

        // Create and solve the QP.
        //
        // q:      Current value of joint angles.
        // ee_vel: Desired task space velocity of end effector.
        // dt:     Control timestep.
        // dq_opt: Optimal values of joint velocities.
        //
        // Returns true if optimization problem was solved successfully
        // (i.e. constraints satisified).
        bool solve(const JointVector& q, const Vector6d& ee_vel,
                   const std::vector<ObstacleModel>& obstacles, double dt,
                   JointVector& dq_opt) {
            /* OBJECTIVE */

            // Minimize velocity objective.
            JointMatrix Q1 = JointMatrix::Identity(); // norm weighting
            JointVector C1 = JointVector::Zero();

            // Manipulability objective.
            JointVector dm;
            JointMatrix Hm = JointMatrix::Zero();
            //linearize_manipulability2(q, dm, Hm);
            linearize_manipulability1(q, dm);

            JointMatrix Q2 = -dt * dt * Hm;
            JointVector C2 = -dt * dm;

            // Avoid joint limits objective.
            JointMatrix Q3 = dt * dt * JointMatrix::Identity();
            Q3(0,0) = 0;
            Q3(1,1) = 0;
            Q3(2,2) = 0;
            JointVector C3 = dt * (2*q - (POSITION_LIMITS_UPPER + POSITION_LIMITS_LOWER));
            C3(0) = 0;
            C3(1) = 0;
            C3(2) = 0;

            // w1=0.1, w2=0, w3=1.0 worked well for line with no orientation control

            double w1 = 1.0;  // velocity
            double w2 = 0.0;  // manipulability
            double w3 = 0.0; // limits

            JointMatrix Q = w1*Q1 + w2*Q2 + w3*Q3;
            JointVector C = w1*C1 + w2*C2 + w3*C3;

            // JointMatrix Q = W;
            // JointVector C = JointVector::Zero();

            /* EQUALITY CONSTRAINTS */

            // Track the reference velocity.
            // TODO relaxation (see Dufour and Suleiman, 2017)
            JacobianMatrix J;
            Kinematics::jacobian(q, J);

            Eigen::Matrix<double, 6, 9> Aeq = J;
            Eigen::Matrix<double, 6, 1> Beq = ee_vel;

            /* INEQUALITY CONSTRAINTS */

            // Eigen::Matrix<double, 1, 0> Aineq, Bineq;

            // Velocity damper ineq constraints
            JointVector dq_lb, dq_ub;
            velocity_damper_limits(q, dq_lb, dq_ub);

            Eigen::Matrix<double, 2*NUM_JOINTS, NUM_JOINTS> A_lim;
            A_lim << -JointMatrix::Identity(), JointMatrix::Identity();

            Eigen::Matrix<double, 2*NUM_JOINTS, 1> b_lim;
            b_lim << -dq_lb, dq_ub;

            // Obstacle constraints.
            Eigen::MatrixXd A_obs;
            Eigen::VectorXd b_obs;
            Eigen::Vector2d pb(q[0], q[1]);
            int num_obs = obstacle_limits(pb, obstacles, A_obs, b_obs);

            int num_ineq = 2*NUM_JOINTS + num_obs;

            Eigen::MatrixXd Aineq(num_ineq, NUM_JOINTS);
            Aineq << A_lim, A_obs;

            Eigen::VectorXd bineq(num_ineq);
            bineq << b_lim, b_obs;

            // Solve the QP.
            // Arguments: # variables, # eq constraints, # ineq constraints
            Eigen::QuadProgDense qp(NUM_JOINTS, 6, num_ineq);
            bool success = qp.solve(Q, C, Aeq, Beq, Aineq, bineq);
            dq_opt = qp.result();

            // Examine objective function values
            double vel_obj = dq_opt.transpose() * Q1 * dq_opt; // want to minimize

            double mi = Kinematics::manipulability(q);
            double mi_obj_quad = dt * dt * dq_opt.transpose() * Hm * dq_opt;
            double mi_obj_lin = dt * dm.dot(dq_opt);

            // This is what actually gets considering in the optimization
            // problem.
            double mi_obj = w1 * (mi_obj_quad + mi_obj_lin); // want to maximize

            publish_state(dq_opt, vel_obj, mi_obj);

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

        // Calculate obstacle avoidance objective.
        int obstacle_limits(const Eigen::Vector2d& pb,
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

        // Returns true if the obstacle is within the influence distance of the
        // base, false otherwise.
        // TODO probably move this to obstacle.h
        bool obstacle_is_close(const Eigen::Vector2d& pb,
                               const ObstacleModel& obstacle) {
            Eigen::Vector2d n = obstacle.centre() - pb;
            double d = n.norm() - BASE_RADIUS - obstacle.radius();
            return d < OBS_INFLUENCE_DIST;
        }

        // Filter out obstacles that are not close enough to influence the
        // optimization problem.
        void filter_obstacles(const Eigen::Vector2d& pb,
                              const std::vector<ObstacleModel>& obstacles,
                              std::vector<ObstacleModel>& close_obstacles) {
            for (int i = 0; i < obstacles.size(); ++i) {
                if (obstacle_is_close(pb, obstacles[i])) {
                    close_obstacles.push_back(obstacles[i]);
                }
            }
        }

    private:
        typedef realtime_tools::RealtimePublisher<mm_msgs::OptimizationState> StatePublisher;
        typedef std::unique_ptr<StatePublisher> StatePublisherPtr;

        // typedef Eigen::Matrix<double, Eigen::Dynamic, 2> MatrixX2d;

        StatePublisherPtr state_pub;

        void publish_state(const JointVector& dq_opt, double vel_obj,
                           double mi_obj) {
            if (state_pub->trylock()) {
                state_pub->msg_.labels = { "vel", "mi" };
                state_pub->msg_.objectives = { vel_obj, mi_obj };
                state_pub->msg_.result = std::vector<double>(dq_opt.data(), dq_opt.data() + dq_opt.size());

                state_pub->msg_.header.stamp = ros::Time::now();
                state_pub->unlockAndPublish();
            }
        }
}; // class IKOptimizer

} // namespace mm

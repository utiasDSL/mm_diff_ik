#pragma once

#include <Eigen/Eigen>
#include <QuadProg.h>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <qpOASES/qpOASES.hpp>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <mm_msgs/OptimizationState.h>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/differentiation.h>

#include "mm_motion_control/obstacle.h"


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


class IKOptimizer {
    public:
        IKOptimizer() {};

        bool init(ros::NodeHandle& nh) {
            state_pub.reset(new StatePublisher(nh, "/optimization_state", 1));
        }

        // Create and solve the QP.
        //
        // q:      Current joint angles.
        // dq:     Current joint velocities.
        // d:      d = pd - f(q)
        // vd:     Desired task space velocity of end effector.
        // dt:     Control timestep.
        // dq_opt: Optimal values of joint velocities.
        //
        // Return value is 0 if the problem was solved successfully. Otherwise,
        // status code indicates a failure in the optmization problem.
        int solve(const JointVector& q, const JointVector& dq,
                  const Vector6d& d, const Vector6d& vd,
                  const std::vector<ObstacleModel>& obstacles, double dt,
                  JointVector& dq_opt) {

            /*** OBJECTIVES ***/

            /* 1. Minimize velocity objective. */

            JointMatrix Q1 = JointMatrix::Identity();
            JointVector C1 = JointVector::Zero();

            // To reduce base movement, increase the base joint weighting.
            // Q1(0,0) = 10;
            // Q1(1,1) = 10;
            // Q1(2,2) = 10;

            /* 2. Manipulability objective. */

            JointVector dm;
            JointMatrix Hm = JointMatrix::Zero();
            //linearize_manipulability2(q, dm, Hm);
            linearize_manipulability1(q, dm);

            JointMatrix Q2 = -dt * dt * Hm;
            JointVector C2 = -dt * dm;

            /* 3. Avoid joint limits objective. */
            // TODO this is experimental

            JointMatrix Q3 = dt * dt * JointMatrix::Identity();
            Q3(0,0) = 0;
            Q3(1,1) = 0;
            Q3(2,2) = 0;
            JointVector C3 = dt * (2*q - (POSITION_LIMITS_UPPER + POSITION_LIMITS_LOWER));
            C3(0) = 0;
            C3(1) = 0;
            C3(2) = 0;

            // w1=0.1, w2=0, w3=1.0 worked well for line with no orientation control

            /* 4. Error minimization objective */

            // (instead of making tracking a constraint)
            // TODO want to split into position and orientation components, to
            // make it easier to test independently

            JacobianMatrix J;
            Kinematics::jacobian(q, J);

            Matrix6d W4 = Matrix6d::Identity();
            W4(3,3) = 0.1;
            W4(4,4) = 0.1;
            W4(5,5) = 0.1;
            JointMatrix Q4 = dt * dt * J.transpose() * W4 * J;
            JointVector C4 = -dt * d.transpose() * W4 * J;

            /* 5. Minimize joint acceleration */

            // Only do numerical differentiation with a non-zero timestep.
            JointMatrix Q5 = JointMatrix::Zero();
            if (dt * dt > 0) {
                Q5 = JointMatrix::Identity() / (dt * dt);
            }
            JointVector C5 = -dq.transpose() * Q5;


            /* Objective weighting */

            double w1 = 1.0; // velocity
            double w2 = 0.0; // manipulability
            double w3 = 0.0; // joint limits
            double w4 = 100.0; // pose error
            double w5 = 0.1; // acceleration

            // TODO obstacle avoidance can also be done here

            JointMatrix Q = w1*Q1 + w2*Q2 + w3*Q3 + w4*Q4 + w5*Q5;
            JointVector C = w1*C1 + w2*C2 + w3*C3 + w4*C4 + w5*C5;


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
            // TODO we may also want this to be an objective
            Eigen::MatrixXd A_obs;
            Eigen::VectorXd b_obs;
            Eigen::Vector2d pb(q[0], q[1]);
            int num_obs = obstacle_limits(pb, obstacles, A_obs, b_obs);


            /*** SOLVE QP ***/

            // Convert to row-major order. Only meaningful for matrices, not
            // vectors.
            Eigen::Matrix<qpOASES::real_t, NUM_JOINTS, NUM_JOINTS, Eigen::RowMajor> H_rowmajor = Q;
            Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_rowmajor = A_obs;

            // Convert eigen data to raw arrays for qpOASES.
            qpOASES::real_t *H = H_rowmajor.data();
            qpOASES::real_t *g = C.data();

            qpOASES::real_t *lb = dq_lb.data();
            qpOASES::real_t *ub = dq_ub.data();

            qpOASES::real_t *A = A_rowmajor.data();
            qpOASES::real_t *ubA = b_obs.data();
            qpOASES::real_t *lbA = NULL;

            qpOASES::int_t nWSR = 10;

            // Solve the QP.
            // qpOASES::QProblemB qp(NUM_JOINTS, num_obs);
            // qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);
            qpOASES::QProblemB qp(NUM_JOINTS);
            qp.init(H, g, lb, ub, nWSR);

            qpOASES::real_t dq_opt_raw[NUM_JOINTS];
            int status = qp.getPrimalSolution(dq_opt_raw);
            dq_opt = Eigen::Map<JointVector>(dq_opt_raw); // map back to eigen


            /*** CALCULATE OBJECTIVE FUNCTION VALUES ***/

            // Examine objective function values
            double vel_obj = dq_opt.transpose() * Q1 * dq_opt; // want to minimize

            double mi = Kinematics::manipulability(q);
            double mi_obj_quad = dt * dt * dq_opt.transpose() * Hm * dq_opt;
            double mi_obj_lin = dt * dm.dot(dq_opt);

            // This is what actually gets considering in the optimization
            // problem.
            double mi_obj = w1 * (mi_obj_quad + mi_obj_lin); // want to maximize

            publish_state(dq_opt, vel_obj, mi_obj);

            return status;
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

        // Filter out obstacles that are not close enough to influence the
        // optimization problem.
        void filter_obstacles(const Eigen::Vector2d& pb,
                              const std::vector<ObstacleModel>& obstacles,
                              std::vector<ObstacleModel>& close_obstacles) {
            for (int i = 0; i < obstacles.size(); ++i) {
                if (obstacles[i].in_range(pb)) {
                    close_obstacles.push_back(obstacles[i]);
                }
            }
        }

    private:
        typedef realtime_tools::RealtimePublisher<mm_msgs::OptimizationState> StatePublisher;
        typedef std::unique_ptr<StatePublisher> StatePublisherPtr;

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

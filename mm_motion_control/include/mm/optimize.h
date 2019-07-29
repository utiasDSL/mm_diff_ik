#pragma once

#include <Eigen/Eigen>
#include <QuadProg.h>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include "mm/mm.h"
#include "mm/kinematics.h"


// eigen-quadprog API:
//
// QuadProgDense(int nrvar, int nreq, int nrineq);
//
// bool solve(const MatrixXd& Q, const VectorXd& C,
// 	const MatrixXd& Aeq, const VectorXd& Beq,
// 	const MatrixXd& Aineq, const VectorXd& Bineq,
// 	bool isDecomp=false);
//


namespace mm {
    class IKOptimizer {
        public:
            IKOptimizer() {};

            // Create and solve the QP.
            // q: Current value of joint angles.
            // ee_vel: Desired task space velocity of end effector.
            // dq_opt: Optimal values of joint velocities.
            bool solve(const QVector& q, const Vector6d& ee_vel,
                       QVector& dq_opt) {
                // Objective
                QMatrix Q = QMatrix::Identity();
                QVector C = QVector::Zero();

                // Constraints
                JacMatrix J;
                Kinematics::jacobian(q, J);

                // No inequality constraints for now.
                Eigen::Matrix<double, 1, 0> Aineq, Bineq;

                // Solve the QP.
                Eigen::QuadProgDense qp(NUM_JOINTS, 6, 0);
                bool ret = qp.solve(Q, C, J, ee_vel, Aineq, Bineq);
                dq_opt = qp.result();

                return ret;
            }
    };
}

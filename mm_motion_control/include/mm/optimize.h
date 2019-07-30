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


namespace mm {
    class IKOptimizer {
        public:
            IKOptimizer() {};

            // Create and solve the QP.
            // q: Current value of joint angles.
            // ee_vel: Desired task space velocity of end effector.
            // dq_opt: Optimal values of joint velocities.
            // Returns true if optimization problem was solved successfully
            // (i.e. constraints satisified).
            bool solve(const QVector& q, const Vector6d& ee_vel,
                       QVector& dq_opt) {
                // Objective
                QMatrix Q = QMatrix::Identity();
                QVector C = QVector::Zero();

                // Equality constraints
                JacMatrix J;
                Kinematics::jacobian(q, J);

                ROS_INFO_STREAM("q = " << q);
                ROS_INFO_STREAM("ee_vel = " << ee_vel);

                Eigen::Matrix<double, 6, 9> Aeq = J;
                Eigen::Matrix<double, 6, 1> Beq = ee_vel;

                // Inequality constraints
                // Eigen::Matrix<double, 1, 0> Aineq, Bineq;

                // Requires -1 <= dq <= 1
                Eigen::Matrix<double, 18, 9> Aineq;
                Aineq << QMatrix::Identity(), -QMatrix::Identity();
                Eigen::Matrix<double, 18, 1> Bineq = Eigen::Matrix<double, 18, 1>::Ones();

                // Solve the QP.
                Eigen::QuadProgDense qp(NUM_JOINTS, 6, 18);
                bool success = qp.solve(Q, C, Aeq, Beq, Aineq, Bineq);
                dq_opt = qp.result();

                // ROS_INFO_STREAM("dq_opt = " << dq_opt);

                return success;
            }
    };
}

#pragma once

#include <Eigen/Eigen>
#include <QuadProg.h>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

//#include "mm/mm.h"
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

// Approximate gradient using central differences.
// f: Function mapping N-dim vector input to scalar output
// h: Step size
// x: Input point
// grad: The gradient, populated by this function.
template<int N>
void approx_gradient(double (*f)(const Eigen::Matrix<double, N, 1>&),
                     double h, const Eigen::Matrix<double, N, 1>& x,
                     Eigen::Matrix<double, N, 1>& grad) {
    typedef Eigen::Matrix<double, N, N> MatrixNd;
    typedef Eigen::Matrix<double, N, 1> VectorNd;

    MatrixNd I = MatrixNd::Identity();

    for (int i = 0; i < N; ++i) {
        VectorNd Ii = I.col(i);
        grad(i) = (f(x + h*Ii) - f(x - h*Ii)) / (2*h);
    }
}


// Approximate Hessian using central differences.
// f: Function mapping N-dim vector input to scalar output
// h: Step size
// x: Input point
// H: The Hessian, populated by this function.
template<int N>
void approx_hessian(double (*f)(const Eigen::Matrix<double, N, 1>&),
                    double h, const Eigen::Matrix<double, N, 1>& x,
                    Eigen::Matrix<double, N, N>& H) {
    typedef Eigen::Matrix<double, N, N> MatrixNd;
    typedef Eigen::Matrix<double, N, 1> VectorNd;

    MatrixNd I = MatrixNd::Identity();

    for (int i = 0; i < N; ++i) {
        VectorNd Ii = I.col(i);
        for (int j = 0; j < N; ++j) {
            VectorNd Ij = I.col(j);
            H(i,j) = (f(x + h*Ii + h*Ij) - f(x + h*Ii - h*Ij)
                    - f(x - h*Ii + h*Ij) + f(x - h*Ii - h*Ij)) / (4*h*h);
        }
    }
}

class IKOptimizer {
    public:
        IKOptimizer() {};

        // Create and solve the QP.
        // q: Current value of joint angles.
        // ee_vel: Desired task space velocity of end effector.
        // dq_opt: Optimal values of joint velocities.
        // Returns true if optimization problem was solved successfully
        // (i.e. constraints satisified).
        // TODO we're going to want dt here
        bool solve(const JointVector& q, const Vector6d& ee_vel,
                   JointVector& dq_opt) {
            // Minimize velocity objective.
            JointMatrix Q = JointMatrix::Identity();
            JointVector C = JointVector::Zero();

            // Manipulability objective.
            // JointVector dm;
            // JointMatrix Hm;
            // linearized_manipulability(q, dm, Hm);

            // Equality constraints: track the reference velocity.
            // TODO relaxation
            JacobianMatrix J;
            Kinematics::jacobian(q, J);

            Eigen::Matrix<double, 6, 9> Aeq = J;
            Eigen::Matrix<double, 6, 1> Beq = ee_vel;

            // Inequality constraints
            // Eigen::Matrix<double, 1, 0> Aineq, Bineq;

            // Requires -1 <= dq <= 1
            Eigen::Matrix<double, 18, 9> Aineq;
            Aineq << JointMatrix::Identity(), -JointMatrix::Identity();
            Eigen::Matrix<double, 18, 1> Bineq = Eigen::Matrix<double, 18, 1>::Ones();

            // Solve the QP.
            Eigen::QuadProgDense qp(NUM_JOINTS, 6, 18);
            bool success = qp.solve(Q, C, Aeq, Beq, Aineq, Bineq);
            dq_opt = qp.result();

            return success;
        }

    private:
        // Linearize manipulability around joint values q, returning the
        // value m, gradient dm, and Hessian Hm.
        void linearized_manipulability(const JointVector& q, JointVector& dm,
                                       JointMatrix& Hm) {
            double h = 1e-5; // Step size
            approx_gradient<NUM_JOINTS>(&Kinematics::manipulability, h, q, dm);
            approx_hessian<NUM_JOINTS>(&Kinematics::manipulability, h, q, Hm);
        }
}; // class IKOptimizer

} // namespace mm

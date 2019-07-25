#include <ros/ros.h>
#include <Eigen/Eigen>
#include <QuadProg.h>

#include <std_msgs/Float64MultiArray.h>

#include "rr/rr.h"
#include "rr/jacobian.h"
#include "rr/Optimize.h"

// ROS online redundancy resolution and inverse kinematics.
//
//
// QuadProgDense(int nrvar, int nreq, int nrineq);
//
// bool solve(const MatrixXd& Q, const VectorXd& C,
// 	const MatrixXd& Aeq, const VectorXd& Beq,
// 	const MatrixXd& Aineq, const VectorXd& Bineq,
// 	bool isDecomp=false);
//

namespace rr {
    void dh_transform(double q, double a, double d, double alpha, Transform& T) {
        double sq = std::sin(q);
        double cq = std::cos(q);
        double salpha = std::sin(alpha);
        double calpha = std::cos(alpha);

        T << cq, -sq*calpha, sq*salpha, a*cq,
             sq,  cq*calpha, cq*salpha, a*sq,
             0,   salpha,    calpha,    d,
             0,   0,         0,         1;
    }


    bool optimize(QVector& q_current, PoseVector& P_desired,
                  PoseVector& dP_desired, QVector& q_opt, QVector& dq_opt) {
        // Objective
        QMatrix Q = QMatrix::Identity();
        QVector C = QVector::Zero();

        // Constraints
        JacMatrix J;
        calc_jacobian(q_current, J);

        Eigen::Matrix<double, 1, 0> Aineq, Bineq;

        // Solve the QP.
        Eigen::QuadProgDense qp(NUM_JOINTS, 6, 0);
        bool r = qp.solve(Q, C, J, dP_desired, Aineq, Bineq);
        dq_opt = qp.result();

        // Currently not optimizing on position level.
        q_opt = QVector::Zero();

        return true;
    }

    bool optimize_cb(rr::Optimize::Request  &req, rr::Optimize::Response &res) {
        // Convert to Eigen.
        QVector q_current(req.q_current.data());
        PoseVector P_desired(req.P_desired.data());
        PoseVector dP_desired(req.dP_desired.data());

        QVector q_opt, dq_opt;

        bool status = optimize(q_current, P_desired, dP_desired, q_opt, dq_opt);

        // Populate response
        res.q_opt = std::vector<double>(q_opt.data(), q_opt.data() + q_opt.size());
        res.dq_opt = std::vector<double>(dq_opt.data(), dq_opt.data() + dq_opt.size());

        return status;
    }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "ik_optimize_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("ik_optimize", rr::optimize_cb);
  ROS_INFO("ik_optimize server started");
  ros::spin();

  return 0;
}


// int main() {
//     Matrix2d Q;
//     Vector2d C;
//     Matrix<double, 3, 2> Aineq;
//     Vector3d Bineq;
//
//     Matrix<double, 1, 0> Aeq, Beq;
//
//     Q <<  1, -1,
//          -1,  2;
//     C << -2, -6;
//     Aineq <<  1, 1,
//              -1, 2,
//               2, 1;
//     Bineq << 2, 2, 3;
//
//     QuadProgDense qp(2, 0, 3);
//     bool r = qp.solve(Q, C, Aeq, Beq, Aineq, Bineq);
//     Vector2d x = qp.result();
//     std::cout << x << std::endl;
// }

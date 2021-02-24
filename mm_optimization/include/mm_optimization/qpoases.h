#pragma once

#include <Eigen/Eigen>
#include <qpOASES/qpOASES.hpp>


namespace mm {
namespace qpoases {


static const int NUM_WSR = 50;

using QPMatrixXd = Eigen::Matrix<qpOASES::real_t,
                                 Eigen::Dynamic,
                                 Eigen::Dynamic,
                                 Eigen::RowMajor>;
using QPVectorXd = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>;

// Data matrices for the QP.
struct QPData {
    QPMatrixXd H;
    QPVectorXd g;

    QPVectorXd lb;
    QPVectorXd ub;

    QPMatrixXd A;
    QPVectorXd lbA;
    QPVectorXd ubA;
};

// State of the QP
// TODO rename to QPSolution?
struct QPState {
    QPVectorXd x;  // Solution.
    double objective;  // Objective value.

    // Detailed status code returned by optimization problem. 0 indicates
    // success (see qpOASES user manual).
    qpOASES::returnValue code;

    // Simple status code indicating status of optimization problem:
    //  0: QP was solved;
    //  1: QP could not be solved within the given number of iterations;
    // -1: QP could not be solved due to an internal error;
    // -2: QP is infeasible and thus could not be solved;
    // -3: QP is unbounded and thus could not be solved.
    int status;
};


class QProblem {
    public:
        QPData data;
        QPState state;
        qpOASES::Options options;

        QProblem(int nv, int nc);

        int solve(Eigen::VectorXd& x);

    private:
        // Number of variables and constraints. These cannot change, so are
        // passed to constructor to indicate the fact.
        int nv, nc;
};


// class SQProblem {
//
// };

} // namespace qpoases
} // namespace mm

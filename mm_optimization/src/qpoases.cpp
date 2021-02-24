#include "mm_optimization/qpoases.h"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <qpOASES/qpOASES.hpp>


namespace mm {
namespace qpoases {



QProblem::QProblem(int nv, int nc) {
    this->nv = nv;
    this->nc = nc;
}


int QProblem::solve(Eigen::VectorXd& x) {
    qpOASES::real_t *H = data.H.data();
    qpOASES::real_t *g = data.g.data();
    qpOASES::real_t *A = data.A.data();

    // Bounds and constraint matrices may be empty; in which case we pass a
    // NULL pointer to the solver.
    qpOASES::real_t *lb;
    if (data.lb.size() > 0) {
        lb = data.lb.data();
    }
    qpOASES::real_t *ub;
    if (data.ub.size() > 0) {
        ub = data.ub.data();
    }

    qpOASES::real_t *ubA;
    if (data.ubA.size() > 0) {
        ubA = data.ubA.data();
    }
    qpOASES::real_t *lbA;
    if (data.lbA.size() > 0) {
        lbA = data.lbA.data();
    }

    qpOASES::int_t nWSR = NUM_WSR;

    // Solve the QP: no warmstarting.
    qpOASES::QProblem qp(nv, nc);
    qp.setOptions(options);
    qpOASES::returnValue ret = qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);
    int status = qpOASES::getSimpleStatus(ret);

    qpOASES::real_t x_arr[nv];
    qp.getPrimalSolution(x_arr);
    x = Eigen::Map<Eigen::VectorXd>(x_arr, nv, 1); // map back to eigen

    // Populate state values
    state.x = x;
    state.objective = qp.getObjVal();
    state.code = ret;
    state.status = status;

    // If the objective is NaN or inf, then something went wrong. Warn and set
    // the returned solution to zero to avoid mishaps.
    if (isnan(state.objective) || isinf(state.objective)) {
        ROS_WARN_STREAM("Objective value = " << state.objective);
        x.setZero(nv, 1);
    }

    // Set to zero if the QP is infeasible.
    if (status) {
        ROS_WARN_STREAM("QP is infeasible.");
        x.setZero(nv, 1);
    }

    return status;
}

} // namespace qpoases
} // namespace mm

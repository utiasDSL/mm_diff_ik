#include "mm_optimization/qpoases.h"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <qpOASES/qpOASES.hpp>

namespace mm {
namespace qpoases {

QProblem::QProblem(int nv, int nc, qpOASES::int_t nWSR) {
  this->nv = nv;
  this->nc = nc;
  this->nWSR = nWSR;
}

int QProblem::solve(Eigen::VectorXd &x) {
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

  // Solve the QP: no warmstarting.
  qpOASES::QProblem qp(nv, nc);
  qp.setOptions(options);
  qpOASES::int_t nWSR_ = nWSR;
  qpOASES::returnValue ret = qp.init(H, g, A, lb, ub, lbA, ubA, nWSR_);
  int status = qpOASES::getSimpleStatus(ret);

  qpOASES::real_t x_arr[nv];
  qp.getPrimalSolution(x_arr);
  x = Eigen::Map<Eigen::VectorXd>(x_arr, nv, 1);  // map back to eigen

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
    status = 1;  // non-zero status to inform the controller
  }

  // Set to zero if the QP is infeasible.
  if (status) {
    ROS_WARN_STREAM("QP is infeasible.");
    x.setZero(nv, 1);
  }

  return status;
}

SQProblem::SQProblem(int nv, int nc, qpOASES::int_t nWSR)
    : QProblem(nv, nc, nWSR), sqp(nv, nc) {}

// TODO would like to reduce the duplication between here and above
int SQProblem::solve(Eigen::VectorXd &x) {
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

  sqp.setOptions(options);  // TODO maybe move elsewhere
  qpOASES::returnValue ret;
  qpOASES::int_t nWSR_ = nWSR;
  if (sqp.isInitialised()) {
    ret = sqp.hotstart(H, g, A, lb, ub, lbA, ubA, nWSR_);
  } else {
    ret = sqp.init(H, g, A, lb, ub, lbA, ubA, nWSR_);
  }
  int status = qpOASES::getSimpleStatus(ret);

  qpOASES::real_t x_arr[nv];
  sqp.getPrimalSolution(x_arr);
  x = Eigen::Map<Eigen::VectorXd>(x_arr, nv, 1);  // map back to eigen

  // Populate state values
  state.x = x;
  state.objective = sqp.getObjVal();
  state.code = ret;
  state.status = status;

  // If the objective is NaN or inf, then something went wrong. Warn and set
  // the returned solution to zero to avoid mishaps.
  if (isnan(state.objective) || isinf(state.objective)) {
    ROS_WARN_STREAM("Objective value = " << state.objective);
    x.setZero(nv, 1);
    status = 1;  // non-zero status to inform the controller
  }

  // Set to zero if the QP is infeasible.
  if (status) {
    ROS_WARN_STREAM("QP is infeasible.");
    x.setZero(nv, 1);
  }

  return status;
}

}  // namespace qpoases
}  // namespace mm

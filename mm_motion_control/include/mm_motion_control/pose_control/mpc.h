#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_control/cartesian/control.h>
#include <mm_kinematics/kinematics.h>
#include <mm_optimization/qpoases.h>

namespace mm {

static const double LOOKAHEAD_TIMESTEP = 0.08;

static const int NUM_HORIZON = 10;  // steps to look ahead
static const int NUM_ITER = 2;      // number of relinearizations in SQP
static const int NUM_WSR = 200;     // max number of working set recalculations

static const int NUM_OPT = NUM_JOINTS * NUM_HORIZON;

class MPController : public CartesianController {
 public:
  MPController() : sqp(NUM_OPT, NUM_OPT, NUM_WSR) {}
  ~MPController() {}

  bool init(ros::NodeHandle& nh, const double hz);

 private:
  typedef Eigen::Matrix<double, NUM_OPT, 1> OptVector;
  typedef Eigen::Matrix<double, NUM_OPT, NUM_OPT> OptWeightMatrix;

  typedef Eigen::Matrix<double, 6 * NUM_HORIZON, 1> OptErrorVector;
  typedef Eigen::Matrix<double, 6 * NUM_HORIZON, 6 * NUM_HORIZON>
      OptErrorWeightMatrix;
  typedef Eigen::Matrix<double, 6 * NUM_HORIZON, NUM_JOINTS * NUM_HORIZON>
      OptLiftedJacobian;

  /** VARIABLES **/

  // Constant matrices.
  OptErrorWeightMatrix Qbar;
  OptWeightMatrix Rbar;
  OptWeightMatrix A;

  OptVector dq_min;
  OptVector dq_max;

  OptVector ddq_min;
  OptVector ddq_max;

  // SQP problem
  qpoases::SQProblem sqp;

  /** FUNCTIONS **/
  int update(const ros::Time& now);
};

}  // namespace mm

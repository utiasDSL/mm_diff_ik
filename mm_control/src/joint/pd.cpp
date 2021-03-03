#include "mm_control/joint/pd.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>

#include <mm_control/control.h>
#include <mm_control/joint/trajectory.h>

namespace mm {

// Maximum joint speed.
const static double MAX_U = 0.2;

bool PDJointController::init(ros::NodeHandle& nh,
                              const double hz,
                              const JointMatrix& Kp) {
  mm::JointController::init(nh, hz);
  this->Kp = Kp;
}

int PDJointController::update(const ros::Time& now) {
  mm::JointTrajectoryPoint Xd;
  trajectory.sample(now, Xd);

  // Proportional control with feedforward velocity.
  mm::JointVector e = Xd.positions - q;
  mm::JointMatrix Binv;
  mm::Kinematics::calc_joint_input_map_inv(q, Binv);
  u = Binv * (Kp * e + Xd.velocities);

  // Bound commands. For base joints, these could be quite large.
  // TODO C++ bound_array function would be useful
  for (int i = 0; i < mm::NUM_JOINTS; ++i) {
    if (u(i) > MAX_U) {
      u(i) = MAX_U;
    } else if (u(i) < -MAX_U) {
      u(i) = -MAX_U;
    }
  }
  return 0;
}

}  // namespace mm

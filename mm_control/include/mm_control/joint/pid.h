#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_control/control.h>
#include <mm_control/joint/control.h>
#include <mm_kinematics/kinematics.h>

namespace mm {

class PIDJointController : public JointController {
 public:
  PIDJointController() {}

  bool init(ros::NodeHandle& nh,
            const double hz,
            const JointMatrix& Kp,
            const JointMatrix& Ki);

  int update(const ros::Time& now);

 protected:
  // Controller gains
  JointMatrix Kp, Ki;

};  // class PIDJointController

}  // namespace mm

#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_control/control.h>
#include <mm_control/joint/control.h>
#include <mm_kinematics/kinematics.h>

namespace mm {

class PDJointController : public JointController {
 public:
  PDJointController() {}

  bool init(ros::NodeHandle& nh,
            const double hz,
            const JointMatrix& Kp);

  int update(const ros::Time& now);

 protected:
  // Proportional gain
  JointMatrix Kp;

};  // class PDJointController

}  // namespace mm

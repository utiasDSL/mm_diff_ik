#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/pose_control/optimizer.h"
#include "mm_motion_control/pose_control/trajectory.h"
#include "mm_motion_control/pose_control/rate.h"
#include "mm_motion_control/pose_control/obstacle.h"


using namespace mm;


int main(int argc, char **argv) {
  // ros::init(argc, argv, "mm_pose_control_node");
  // ros::NodeHandle nh;
  //
  double t = 0;
  double dt = CONTROL_TIMESTEP;

  JointVector q;
  q << 1.13463802, -1.09827181,  0.01214866,
       0.07124607, -2.71000883, -1.00152829, -2.36405747, -1.51027803, 1.57077014;

  JointVector dq; // = JointVector::Zero();
  dq << 0.12205255, -0.00169602, 0.00266732,
        0.00127832, -0.01494353, 0.0198267, -0.0037199, 0.0036432, 0;

  double fd = 0;
  Eigen::Vector3d f = Eigen::Vector3d::Zero();
  Eigen::Vector3d pc = Eigen::Vector3d::Zero();
  std::vector<ObstacleModel> obstacles;

  // stationary trajectory
  Eigen::Affine3d w_T_tool;
  Kinematics::calc_w_T_tool(q, w_T_tool);
  Eigen::Quaterniond Qe(w_T_tool.rotation());
  Eigen::Vector3d pe = w_T_tool.translation();
  pe(0) += 0.1;

  PoseTrajectory trajectory;
  trajectory.stay_at(pe, Qe);

  JointVector u;

  IKOptimizer optimizer;
  optimizer.init();
  optimizer.solve(t, trajectory, q, dq, fd, f, pc, obstacles, dt, u);
  ROS_INFO_STREAM("u = " << u);

  return 0;
}

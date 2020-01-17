#include "mm_motion_control/pose_control/controller.h"

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>

#include <mm_msgs/PoseTrajectory.h>
#include <mm_msgs/PoseControlState.h>
#include <mm_msgs/Obstacles.h>
#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/util/messages.h"
#include "mm_motion_control/pose_control/optimizer.h"
#include "mm_motion_control/pose_control/trajectory.h"
#include "mm_motion_control/pose_control/pose_error.h"
#include "mm_motion_control/pose_control/obstacle.h"


namespace mm {


bool IKController::init() {
    optimizer.init();
    tick();
    return true;
}


void IKController::tick() {
    time_prev = ros::Time::now().toSec();
}


int IKController::update(const Eigen::Vector3d& pos_des,
                         const Eigen::Quaterniond& quat_des,
                         const JointVector& q_act, const JointVector& dq_act,
                         const std::vector<ObstacleModel> obstacles,
                         JointVector& dq_cmd) {
    // Update time.
    ros::Time now = ros::Time::now();
    double dt = now.toSec() - time_prev;
    time_prev = now.toSec();

    // Optimize to solve IK problem.
    return optimizer.solve(pos_des, quat_des, q_act, dq_act, obstacles, dt, dq_cmd);
}

} // namespace mm

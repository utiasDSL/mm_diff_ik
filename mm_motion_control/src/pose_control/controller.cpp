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

#include "mm_motion_control/pose_control/optimizer.h"
#include "mm_motion_control/pose_control/trajectory.h"
#include "mm_motion_control/pose_control/pose_error.h"
#include "mm_motion_control/pose_control/obstacle.h"
#include "mm_motion_control/pose_control/controller.h"


namespace mm {


// Populate Pose message from Eigen types.
void pose_msg_from_eigen(const Eigen::Vector3d& p, const Eigen::Quaterniond& q,
                         geometry_msgs::Pose& msg) {
    msg.position.x = p(0);
    msg.position.y = p(1);
    msg.position.z = p(2);

    msg.orientation.w = q.w();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
}


bool IKController::init(ros::NodeHandle& nh, Eigen::Matrix3d& Kv,
                        Eigen::Matrix3d& Kw) {
    this->Kv = Kv;
    this->Kw = Kw;
    time_prev = ros::Time::now().toSec();

    optimizer.init(nh);
    state_pub.reset(new StatePublisher(nh, "/mm_pose_state", 1));
}


void IKController::tick() {
    time_prev = ros::Time::now().toSec();
}


int IKController::update(const Eigen::Vector3d& pos_des, const Eigen::Quaterniond& quat_des,
                         const Eigen::Vector3d& v_ff, const Eigen::Vector3d& w_ff,
                         const JointVector& q_act, const JointVector& dq_act,
                         const std::vector<ObstacleModel> obstacles,
                         JointVector& dq_cmd) {
    // Calculate actual pose using forward kinematics.
    Eigen::Affine3d ee_pose_act;
    Kinematics::forward(q_act, ee_pose_act);

    // Position error.
    Eigen::Vector3d pos_act = ee_pose_act.translation();
    Eigen::Vector3d pos_err = pos_des - pos_act;

    // TODO there is a weird duplication of effort here in some calculations
    Eigen::Vector3d rot_err;
    rotation_error(quat_des, q_act, rot_err);

    // Orientation error (see pg. 140 of Siciliano et al., 2010).
    Eigen::Quaterniond quat_act(ee_pose_act.rotation());
    Eigen::Quaterniond quat_err = quat_des * quat_act.inverse();

    Vector6d d;
    d << pos_err, rot_err;

    // Velocity command in task space: P control with velocity feedforward.
    // At the moment we assume zero rotational feedforward.
    Eigen::Vector3d v = Kv * pos_err + v_ff;
    Eigen::Vector3d w = Kw * rot_err + w_ff;

    Vector6d vel_cmd;
    vel_cmd << v, w;

    // Update time.
    ros::Time now = ros::Time::now();
    double dt = now.toSec() - time_prev;
    time_prev = now.toSec();

    // Optimize to solve IK problem.
    int status = optimizer.solve(pos_des, quat_des, q_act, dq_act, vel_cmd, obstacles, dt, dq_cmd);

    publish_state(now, pos_act, quat_act, pos_des, quat_des, pos_err, quat_err, v_ff);

    return status;
}


void IKController::publish_state(const ros::Time& time,
                                 const Eigen::Vector3d& pos_act,
                                 const Eigen::Quaterniond& quat_act,
                                 const Eigen::Vector3d& pos_des,
                                 const Eigen::Quaterniond& quat_des,
                                 const Eigen::Vector3d& pos_err,
                                 const Eigen::Quaterniond& quat_err,
                                 const Eigen::Vector3d& vel_des) {
    if (state_pub->trylock()) {
        geometry_msgs::Pose pose_act_msg, pose_des_msg, pose_err_msg;

        pose_msg_from_eigen(pos_act, quat_act, pose_act_msg);
        pose_msg_from_eigen(pos_des, quat_des, pose_des_msg);
        pose_msg_from_eigen(pos_err, quat_err, pose_err_msg);

        state_pub->msg_.actual = pose_act_msg;
        state_pub->msg_.desired = pose_des_msg;
        state_pub->msg_.error = pose_err_msg;

        state_pub->msg_.twist_desired.linear.x = vel_des(0);
        state_pub->msg_.twist_desired.linear.y = vel_des(1);
        state_pub->msg_.twist_desired.linear.z = vel_des(2);

        state_pub->msg_.header.frame_id = "world";
        state_pub->msg_.header.stamp = time;
        state_pub->unlockAndPublish();
    }
}

} // namespace mm

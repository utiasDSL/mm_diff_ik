#include "mm_motion_control/pose_control/manager.h"

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
#include <mm_msgs/ForceInfo.h>
#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/util/messages.h"
#include "mm_motion_control/pose_control/optimizer.h"
#include "mm_motion_control/pose_control/trajectory.h"
#include "mm_motion_control/pose_control/pose_error.h"
#include "mm_motion_control/pose_control/obstacle.h"
#include "mm_motion_control/pose_control/controller.h"


namespace mm {


bool IKControllerManager::init(ros::NodeHandle& nh) {
    state_pub.reset(new StatePublisher(nh, "/mm_pose_state", 1));

    pose_traj_sub = nh.subscribe("/trajectory/poses", 1,
            &IKControllerManager::pose_traj_cb, this);

    point_traj_sub = nh.subscribe("/trajectory/point", 1,
            &IKControllerManager::point_traj_cb, this);

    mm_joint_states_sub = nh.subscribe("/mm_joint_states", 1,
            &IKControllerManager::mm_joint_states_cb, this);

    force_position_offset_sub = nh.subscribe("/force_control/position_offset",
            1, &IKControllerManager::pos_offset_cb, this);

    force_sub = nh.subscribe("/force/info",
            1, &IKControllerManager::force_cb, this);

    obstacle_sub = nh.subscribe("/obstacles", 1,
                                &IKControllerManager::obstacle_cb, this);

    ur10_joint_vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ur_driver/joint_speed", 1);
    rb_joint_vel_pub = nh.advertise<geometry_msgs::Twist>(
            "/ridgeback_velocity_controller/cmd_vel", 1);

    controller.init(ros::Time::now().toSec());

    q_act = JointVector::Zero();
    dq_act = JointVector::Zero();

    // tau value taken from original parallel force controller
    force = Eigen::Vector3d::Zero();
    first_contact = false;

    traj_active = false;
}


// Control loop.
void IKControllerManager::loop(const double hz) {
    ros::Rate rate(hz);
    ROS_INFO("Control loop started, waiting for trajectory...");

    while (ros::ok()) {
        ros::spinOnce();

        double t = ros::Time::now().toSec();

        if (!traj_active) {
            publish_joint_speeds(JointVector::Zero());
            controller.set_time(t);
            rate.sleep();
            continue;
        }

        if (trajectory.done(t)) {
            ROS_INFO("Trajectory ended.");
            traj_active = false;
            continue;
        }

        JointVector dq_cmd = JointVector::Zero();
        int status = controller.update(t, trajectory, q_act, dq_act, force,
                                       obstacles, dq_cmd);

        // Return value is 0 is successful, non-zero otherwise.
        if (status) {
            // Send zero velocity command if optimization fails (the velocity
            // commands are garbage in this case).
            ROS_WARN_STREAM("Optimization failed with status = " << status);
            publish_joint_speeds(JointVector::Zero());
        } else {
            publish_joint_speeds(dq_cmd);
        }

        Eigen::Affine3d Td;
        Vector6d twist;
        trajectory.sample(t, Td, twist);
        publish_robot_state(Td, q_act, dq_act);

        rate.sleep();
    }
}


void IKControllerManager::publish_joint_speeds(const JointVector& dq_cmd) {
    trajectory_msgs::JointTrajectory traj_arm;
    geometry_msgs::Twist twist_base;

    joint_speed_msgs(dq_cmd, traj_arm, twist_base);

    ur10_joint_vel_pub.publish(traj_arm);
    rb_joint_vel_pub.publish(twist_base);
}


// An entire trajectory is sent
// Single point method is a special case
void IKControllerManager::pose_traj_cb(const mm_msgs::PoseTrajectory& msg) {
    trajectory.follow(msg);
    traj_active = true;
    ROS_INFO("Trajectory started.");
}


void IKControllerManager::point_traj_cb(const geometry_msgs::PoseStamped& msg) {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    pose_msg_to_eigen(msg.pose, p, q);
    trajectory.stay_at(p, q);
    traj_active = true;
    ROS_INFO("Maintaining current pose.");
}


void IKControllerManager::mm_joint_states_cb(const sensor_msgs::JointState& msg) {
    for (int i = 0; i < mm::NUM_JOINTS; ++i) {
        q_act(i) = msg.position[i];
        dq_act(i) = msg.velocity[i];
    }
}


void IKControllerManager::pos_offset_cb(const geometry_msgs::Vector3Stamped& msg) {
    Eigen::Vector3d p_off;
    p_off << msg.vector.x, msg.vector.y, msg.vector.z;
    trajectory.offset(p_off);
}


void IKControllerManager::force_cb(const mm_msgs::ForceInfo& msg) {
    force(0) = msg.force_world.x;
    force(1) = msg.force_world.y;
    force(2) = msg.force_world.z;

    // If this is the first time contact is made, switch to maintaining the
    // current pose.
    if (first_contact != msg.first_contact) {
        Eigen::Affine3d w_T_e;
        Kinematics::calc_w_T_e(q_act, w_T_e);
        Eigen::Vector3d p = w_T_e.translation();
        Eigen::Quaterniond q(w_T_e.rotation());
        trajectory.stay_at(p, q);

        first_contact = msg.first_contact;
    }
}


void IKControllerManager::obstacle_cb(const mm_msgs::Obstacles& msg) {
    obstacles.clear();
    for (int i = 0; i < msg.obstacles.size(); ++i) {
        obstacles.push_back(ObstacleModel(msg.obstacles[i]));
    }
}


void IKControllerManager::publish_robot_state(const Eigen::Affine3d& Td,
                                              const JointVector& q,
                                              const JointVector& dq) {
    // Desired pose.
    Eigen::Vector3d pos_des = Td.translation();
    Eigen::Quaterniond quat_des(Td.rotation());

    // Actual pose.
    Eigen::Affine3d w_T_e;
    Kinematics::calc_w_T_e(q, w_T_e);
    Eigen::Vector3d pos_act = w_T_e.translation();
    Eigen::Quaterniond quat_act(w_T_e.rotation());

    // Error.
    Eigen::Vector3d pos_err = pos_des - pos_act;
    Eigen::Quaterniond quat_err = quat_des * quat_act.inverse();

    if (state_pub->trylock()) {
        geometry_msgs::Pose pose_act_msg, pose_des_msg, pose_err_msg;

        pose_msg_from_eigen(pos_act, quat_act, pose_act_msg);
        pose_msg_from_eigen(pos_des, quat_des, pose_des_msg);
        pose_msg_from_eigen(pos_err, quat_err, pose_err_msg);

        state_pub->msg_.actual = pose_act_msg;
        state_pub->msg_.desired = pose_des_msg;
        state_pub->msg_.error = pose_err_msg;

        state_pub->msg_.header.frame_id = "world";
        state_pub->msg_.header.stamp = ros::Time::now();
        state_pub->unlockAndPublish();
    }
}

} // namespace mm

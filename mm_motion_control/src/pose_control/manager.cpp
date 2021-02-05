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
#include <std_msgs/Float64.h>

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

    force_info_sub = nh.subscribe("/force/info",
            1, &IKControllerManager::force_info_cb, this);

    force_des_sub = nh.subscribe("/force/desired",
            1, &IKControllerManager::force_des_cb, this);

    obstacle_sub = nh.subscribe("/obstacles", 1,
                                &IKControllerManager::obstacle_cb, this);

    ur10_joint_vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ur_driver/joint_speed", 1);
    rb_joint_vel_pub = nh.advertise<geometry_msgs::Twist>(
            "/ridgeback_velocity_controller/cmd_vel", 1);

    double now = ros::Time::now().toSec();
    controller.init(now);

    q = JointVector::Zero();
    dq = JointVector::Zero();
    u = JointVector::Zero();

    fd = 0;
    force = Eigen::Vector3d::Zero();
    first_contact = false;
    pc = Eigen::Vector3d::Zero();

    traj_active = false;

    last_joint_state_time = now;
}


// Control loop.
void IKControllerManager::loop(const double hz) {
    ros::Rate rate(hz);
    ROS_INFO("Control loop started, waiting for trajectory...");

    while (ros::ok()) {
        ros::spinOnce();

        ros::Time now = ros::Time::now();
        double t = now.toSec();

        // Do nothing if there is no trajectory active.
        if (!traj_active) {
            publish_joint_speeds(now);
            controller.set_time(t);
            rate.sleep();
            continue;
        }

        // When the trajectory ends, make sure to publish a zero velocity in
        // the case the trajectory itself doesn't end at zero.
        if (trajectory.done(t)) {
            ROS_INFO("Trajectory ended.");
            traj_active = false;
            u = JointVector::Zero();
            publish_joint_speeds(now);
            continue;
        }

        // Integrate using the model to get the best estimate of the state.
        // TODO: not sure if it would be better to use u instead of dq
        double dt = t - last_joint_state_time;
        q = q + dt * dq;
        last_joint_state_time = t;

        int status = controller.update(t, trajectory, q, dq, fd, force, pc,
                                       obstacles, u);

        // print but then publish 0
        // ROS_INFO_STREAM("u = " << u);
        // u = JointVector::Zero();

        // Return value is 0 if successful, non-zero otherwise.
        if (status) {
            // Send zero velocity command if optimization fails (the velocity
            // commands are garbage in this case).
            ROS_WARN_STREAM("Optimization failed with status = " << status);
            u = JointVector::Zero();
        }

        publish_joint_speeds(now);
        publish_robot_state(now);

        rate.sleep();
    }
}


void IKControllerManager::publish_joint_speeds(const ros::Time& now) {
    trajectory_msgs::JointTrajectory traj_arm_msg;
    geometry_msgs::Twist twist_base_msg;

    joint_speed_msgs(u, traj_arm_msg, twist_base_msg);

    traj_arm_msg.header.stamp = now;

    ur10_joint_vel_pub.publish(traj_arm_msg);
    rb_joint_vel_pub.publish(twist_base_msg);
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
    last_joint_state_time = msg.header.stamp.toSec();
    for (int i = 0; i < mm::NUM_JOINTS; ++i) {
        q(i) = msg.position[i];
        dq(i) = msg.velocity[i];
    }
}


void IKControllerManager::pos_offset_cb(const geometry_msgs::Vector3Stamped& msg) {
    Eigen::Vector3d p_off;
    p_off << msg.vector.x, msg.vector.y, msg.vector.z;
    trajectory.offset(p_off);
}


void IKControllerManager::force_info_cb(const mm_msgs::ForceInfo& msg) {
    force(0) = msg.force_world.x;
    force(1) = msg.force_world.y;
    force(2) = msg.force_world.z;

    // If this is the first time contact is made, switch to maintaining the
    // current pose.
    if (first_contact != msg.first_contact) {
        Eigen::Affine3d w_T_tool;
        Kinematics::calc_w_T_tool(q, w_T_tool);
        Eigen::Vector3d p = w_T_tool.translation();
        Eigen::Quaterniond q(w_T_tool.rotation());
        trajectory.stay_at(p, q);
        // traj_active = true;

        // Set the initial contact point.
        pc = p;

        first_contact = msg.first_contact;
        ROS_INFO("Maintaining current pose.");
    }
}


void IKControllerManager::force_des_cb(const std_msgs::Float64 msg) {
    fd = msg.data;
    ROS_INFO_STREAM("Set desired force = " << fd);
}


void IKControllerManager::obstacle_cb(const mm_msgs::Obstacles& msg) {
    obstacles.clear();
    for (int i = 0; i < msg.obstacles.size(); ++i) {
        obstacles.push_back(ObstacleModel(msg.obstacles[i]));
    }
}


void IKControllerManager::publish_robot_state(const ros::Time& now) {
    // ros::Time now = ros::Time::now();
    double t = now.toSec();

    // Actual pose.
    Eigen::Affine3d w_T_tool;
    Kinematics::calc_w_T_tool(q, w_T_tool);
    Eigen::Vector3d pos_act = w_T_tool.translation();
    Eigen::Matrix3d Re = w_T_tool.rotation();
    Eigen::Quaterniond quat_act(Re);

    // Sample desired trajectory.
    Eigen::Affine3d Td;
    Vector6d Vd;
    trajectory.sample(t, Td, Vd);

    // Desired pose.
    Eigen::Vector3d pos_des = Td.translation();
    Eigen::Matrix3d Rd = Td.rotation();
    Eigen::Quaterniond quat_des(Rd);

    // Error.
    Eigen::Vector3d pos_err = pos_des - pos_act;
    Eigen::Quaterniond quat_err = quat_des * quat_act.inverse();
    Eigen::Vector3d rot_err;
    calc_rotation_error(Rd, Re, rot_err);

    if (state_pub->trylock()) {
        geometry_msgs::Pose pose_act_msg, pose_des_msg, pose_err_msg;
        geometry_msgs::Twist Vd_msg;

        pose_msg_from_eigen(pos_act, quat_act, pose_act_msg);
        pose_msg_from_eigen(pos_des, quat_des, pose_des_msg);
        pose_msg_from_eigen(pos_err, quat_err, pose_err_msg);

        twist_msg_from_eigen(Vd, Vd_msg);

        // Pose.
        state_pub->msg_.actual = pose_act_msg;
        state_pub->msg_.desired = pose_des_msg;
        state_pub->msg_.error = pose_err_msg;

        // Twist.
        state_pub->msg_.twist_desired = Vd_msg;

        // Rotation error used for control purposes (not necessarily the same
        // as the actual orientation error).
        state_pub->msg_.rotation_error.x = rot_err(0);
        state_pub->msg_.rotation_error.y = rot_err(1);
        state_pub->msg_.rotation_error.z = rot_err(2);

        // Joint space.
        state_pub->msg_.q = std::vector<double>(q.data(), q.data() + q.size());
        state_pub->msg_.dq = std::vector<double>(dq.data(), dq.data() + dq.size());
        state_pub->msg_.u = std::vector<double>(u.data(), u.data() + u.size());

        state_pub->msg_.header.frame_id = "world";
        state_pub->msg_.header.stamp = now;
        state_pub->unlockAndPublish();
    }
}

} // namespace mm

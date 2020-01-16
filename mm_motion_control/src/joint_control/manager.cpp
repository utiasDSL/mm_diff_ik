#include "mm_motion_control/joint_control/manager.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <mm_msgs/conversions.h>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/joint_control/controller.h"


namespace mm {


bool JointControllerManager::init(ros::NodeHandle& nh) {
    mm_joint_states_sub = nh.subscribe("/mm_joint_states", 1,
            &JointControllerManager::mm_joint_states_cb, this);

    ur10_joint_vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ur_driver/joint_speed", 1);
    rb_joint_vel_pub = nh.advertise<geometry_msgs::Twist>(
            "/ridgeback_velocity_controller/cmd_vel", 1);

    q_act = JointVector::Zero();
    q_des = HOME;

    joint_state_rec = false;

    controller.init(K);
}


void JointControllerManager::loop(const double hz) {
    ros::Rate rate(hz);

    // Wait until we get at joint state message to know the actual
    // joint positions.
    while (ros::ok() && !joint_state_rec) {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        ros::spinOnce();

        JointVector dq_cmd;
        if (!controller.update(q_des, q_act, dq_cmd)) {
            break;
        }

        // Bound commands for base joints, which could be quite large.
        for (int i = 0; i < 3; ++i) {
            if (dq_cmd(i) > MAX_DQ) {
                dq_cmd(i) = MAX_DQ;
            } else if (dq_cmd(i) < -MAX_DQ) {
                dq_cmd(i) = -MAX_DQ;
            }
        }

        publish_joint_speeds(dq_cmd);

        rate.sleep();
    }
}


void JointControllerManager::publish_joint_speeds(const JointVector& dq_cmd) {
    trajectory_msgs::JointTrajectory traj_arm;
    geometry_msgs::Twist twist_base;

    joint_speed_msgs(dq_cmd, traj_arm, twist_base);

    ur10_joint_vel_pub.publish(traj_arm);
    rb_joint_vel_pub.publish(twist_base);
}


void JointControllerManager::mm_joint_states_cb(const sensor_msgs::JointState& msg) {
    for (int i = 0; i < mm::NUM_JOINTS; ++i) {
        q_act(i) = msg.position[i];
    }
    joint_state_rec = true;
}

} // namespace mm

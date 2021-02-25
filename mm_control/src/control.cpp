#include "mm_control/control.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <mm_kinematics/kinematics.h>

#include "mm_control/util/messages.h"


namespace mm {

MMController::MMController() {}
MMController::~MMController() {}


bool MMController::init(ros::NodeHandle& nh, const double hz) {
    this->hz = hz;
    dt = 1.0 / hz;

    did_become_unsafe = false;

    q = JointVector::Zero();
    dq = JointVector::Zero();
    u = JointVector::Zero();

    joint_state_rec = false;
    last_joint_state_time = ros::Time::now().toSec();

    mm_joint_states_sub = nh.subscribe("/mm_joint_states", 1,
            &MMController::mm_joint_states_cb, this);

    ur10_joint_vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ur_driver/joint_speed", 1);
    rb_joint_vel_pub = nh.advertise<geometry_msgs::Twist>(
            "/ridgeback_velocity_controller/cmd_vel", 1);
}


void MMController::publish_joint_speeds(const ros::Time& now) {
    trajectory_msgs::JointTrajectory traj_arm_msg;
    geometry_msgs::Twist twist_base_msg;

    joint_speed_msgs(u, traj_arm_msg, twist_base_msg);

    traj_arm_msg.header.stamp = now;

    ur10_joint_vel_pub.publish(traj_arm_msg);
    rb_joint_vel_pub.publish(twist_base_msg);
}


void MMController::mm_joint_states_cb(const sensor_msgs::JointState& msg) {
    last_joint_state_time = msg.header.stamp.toSec();
    for (int i = 0; i < mm::NUM_JOINTS; ++i) {
        q(i) = msg.position[i];
        dq(i) = msg.velocity[i];
    }
    joint_state_rec = true;
}

} // namespace mm

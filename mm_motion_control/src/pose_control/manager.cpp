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
#include "mm_motion_control/pose_control/manager.h"


namespace mm {


// Controller gain.
Eigen::Matrix3d LINEAR_GAIN = Eigen::Matrix3d::Identity();
Eigen::Matrix3d ROTATIONAL_GAIN = Eigen::Matrix3d::Identity();


bool IKControllerManager::init(ros::NodeHandle& nh) {
    pose_traj_sub = nh.subscribe("/trajectory/poses", 1,
            &IKControllerManager::pose_traj_cb, this);

    point_traj_sub = nh.subscribe("/trajectory/point", 1,
            &IKControllerManager::point_traj_cb, this);

    mm_joint_states_sub = nh.subscribe("/mm_joint_states", 1,
            &IKControllerManager::mm_joint_states_cb, this);

    force_position_offset_sub = nh.subscribe("/force_control/position_offset",
            1, &IKControllerManager::pos_offset_cb, this);

    obstacle_sub = nh.subscribe("/obstacles", 1, &IKControllerManager::obstacle_cb,
                                this);

    ur10_joint_vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1);
    rb_joint_vel_pub = nh.advertise<geometry_msgs::Twist>("/ridgeback_velocity_controller/cmd_vel", 1);

    controller.init(nh, LINEAR_GAIN, ROTATIONAL_GAIN);

    q_act = JointVector::Zero();
    dq_act = JointVector::Zero();
    pos_offset = Eigen::Vector3d::Zero();

    traj_active = false;
}


// Control loop.
void IKControllerManager::loop(const double hz) {
    ros::Rate rate(hz);
    ROS_INFO("Control loop started, waiting for trajectory...");

    while (ros::ok()) {
        ros::spinOnce();

        if (!traj_active) {
            publish_joint_speeds(JointVector::Zero());
            controller.tick();
            rate.sleep();
            continue;
        }

        // Sample interpolated trajectory.
        Eigen::Quaterniond quat_des;
        Eigen::Vector3d pos_des, v_ff, w_ff;
        double now = ros::Time::now().toSec();

        if (!trajectory.sample(now, pos_des, v_ff, quat_des, w_ff)) {
            ROS_INFO("Trajectory ended.");
            traj_active = false;
            continue;
        }

        // Add offset generated by force control to the desired position.
        pos_des += pos_offset;

        JointVector dq_cmd = JointVector::Zero();
        int status = controller.update(pos_des, quat_des, v_ff, w_ff, q_act,
                                       dq_act, obstacles, dq_cmd);
        if (status) {
            // Send zero velocity command if optimization fails (the velocity
            // commands are garbage in this case).
            ROS_WARN("Optimization failed");
            publish_joint_speeds(JointVector::Zero());
            rate.sleep();
            continue;
        }

        publish_joint_speeds(dq_cmd);

        rate.sleep();
    }
}


void IKControllerManager::publish_joint_speeds(const JointVector& dq_cmd) {
    // Split into base and arm joints to send out.
    Eigen::Vector3d dq_cmd_rb = dq_cmd.topRows<3>();
    Vector6d dq_cmd_ur10 = dq_cmd.bottomRows<6>();

    // Convert to JointTrajectory message with a single point (i.e.
    // velocity servoing) to publish to UR10.
    trajectory_msgs::JointTrajectoryPoint point;
    point.velocities = std::vector<double>(
            dq_cmd_ur10.data(), dq_cmd_ur10.data() + dq_cmd_ur10.size());
    trajectory_msgs::JointTrajectory traj_ur10;
    traj_ur10.points.push_back(point);

    // Base twist.
    geometry_msgs::Twist twist_rb;
    twist_rb.linear.x = dq_cmd_rb(0);
    twist_rb.linear.y = dq_cmd_rb(1);
    twist_rb.angular.z = dq_cmd_rb(2);

    ur10_joint_vel_pub.publish(traj_ur10);
    rb_joint_vel_pub.publish(twist_rb);
}


void IKControllerManager::update_forward_kinematics() {
    Kinematics::calc_w_T_b(q_act, w_T_b_act);
    Kinematics::calc_w_T_e(q_act, w_T_e_act);

    Kinematics::forward_vel(q_act, dq_act, dw_T_e_act);
}


// An entire trajectory is sent
// Single point method is a special case
void IKControllerManager::pose_traj_cb(const mm_msgs::PoseTrajectory& msg) {
    trajectory.follow(msg);
    traj_active = true;
    ROS_INFO("Trajectory started.");
}

void IKControllerManager::point_traj_cb(const geometry_msgs::PoseStamped& msg) {
    trajectory.stay_at(msg.pose);
    traj_active = true;
    ROS_INFO("Maintaining current pose.");
}


void IKControllerManager::mm_joint_states_cb(const sensor_msgs::JointState& msg) {
    for (int i = 0; i < mm::NUM_JOINTS; ++i) {
        q_act(i) = msg.position[i];
        dq_act(i) = msg.velocity[i];
    }
    update_forward_kinematics();
}


void IKControllerManager::pos_offset_cb(const geometry_msgs::Vector3Stamped& msg) {
    pos_offset(0) = msg.vector.x;
    pos_offset(1) = msg.vector.y;
    pos_offset(2) = msg.vector.z;
}


void IKControllerManager::obstacle_cb(const mm_msgs::Obstacles& msg) {
    obstacles.clear();
    for (int i = 0; i < msg.obstacles.size(); ++i) {
        obstacles.push_back(ObstacleModel(msg.obstacles[i]));
    }
}

} // namespace mm

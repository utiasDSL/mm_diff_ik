#include <Eigen/Eigen>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <mm_msgs/PoseTrajectoryPoint.h>
#include <mm_msgs/PoseTrajectory.h>
#include <mm_msgs/PoseControlState.h>
#include <mm_kinematics/kinematics.h>
#include <mm_math_util/wrap.h>

#include "mm_motion_control/optimize.h"
#include "mm_motion_control/interp.h"
#include "mm_motion_control/control.h"


using namespace Eigen;


namespace mm {

// Controller gain.
Matrix3d LINEAR_GAIN = Matrix3d::Identity();
Matrix3d ROTATIONAL_GAIN = 0.05*Matrix3d::Identity();


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


bool IKController::init(ros::NodeHandle& nh, Matrix3d& Kv, Matrix3d& Kw) {
    this->Kv = Kv;
    this->Kw = Kw;
    time_prev = ros::Time::now().toSec();

    optimizer.init(nh);
    state_pub.reset(new StatePublisher(nh, "/mm_pose_state", 1));
}


void IKController::tick() {
    time_prev = ros::Time::now().toSec();
}


bool IKController::update(const Vector3d& pos_des, const Quaterniond& quat_des,
                          const Vector3d& v_ff, const Vector3d& w_ff,
                          const JointVector& q_act, JointVector& dq_cmd) {
    // Calculate actual pose using forward kinematics.
    Affine3d ee_pose_act;
    Kinematics::forward(q_act, ee_pose_act);

    // Position error.
    Vector3d pos_act = ee_pose_act.translation();
    Vector3d pos_err = pos_des - pos_act;

    // Orientation error (see pg. 140 of Siciliano et al., 2010).
    Eigen::Quaterniond quat_act(ee_pose_act.rotation());
    Eigen::Quaterniond quat_err = quat_des * quat_act.inverse();

    // Keep quaternion positive. Otherwise, we can get in a situation where it
    // flips between the negative and positive versions and doesn't go
    // anywhere.
    if (quat_err.w() < 0) {
        quat_err.coeffs() *= -1;
    }
    Vector3d rot_err;
    rot_err << quat_err.x(), quat_err.y(), quat_err.z();

    // Eigen::AngleAxisd aa(quat_err);
    // Vector3d rot_err = aa.axis() * wrap_to_pi(aa.angle());
    // ROS_INFO_STREAM(rot_err);

    // Velocity command in task space: P control with velocity feedforward.
    // At the moment we assume zero rotational feedforward.
    Vector3d v = Kv * pos_err + v_ff;
    Vector3d w = Kw * rot_err + w_ff;

    Vector6d vel_cmd;
    vel_cmd << v, w;

    // Update time.
    ros::Time now = ros::Time::now();
    double dt = now.toSec() - time_prev;
    time_prev = now.toSec();

    // Optimize to solve IK problem.
    bool success = optimizer.solve(q_act, vel_cmd, dt, dq_cmd);

    publish_state(now, pos_act, quat_act, pos_des, quat_des, pos_err, quat_err);

    return success;
}


void IKController::publish_state(const ros::Time& time,
                                 const Vector3d& pos_act,
                                 const Quaterniond& quat_act,
                                 const Vector3d& pos_des,
                                 const Quaterniond& quat_des,
                                 const Vector3d& pos_err,
                                 const Quaterniond& quat_err) {
    if (state_pub->trylock()) {
        geometry_msgs::Pose pose_act_msg, pose_des_msg, pose_err_msg;

        pose_msg_from_eigen(pos_act, quat_act, pose_act_msg);
        pose_msg_from_eigen(pos_des, quat_des, pose_des_msg);
        pose_msg_from_eigen(pos_err, quat_err, pose_err_msg);

        state_pub->msg_.actual = pose_act_msg;
        state_pub->msg_.desired = pose_des_msg;
        state_pub->msg_.error = pose_err_msg;

        state_pub->msg_.header.frame_id = "world";
        state_pub->msg_.header.stamp = time;
        state_pub->unlockAndPublish();
    }
}


bool IKControlNode::init(ros::NodeHandle& nh) {
    pose_cmd_sub = nh.subscribe("/pose_cmd", 1,
            &IKControlNode::pose_cmd_cb, this);

    pose_traj_sub = nh.subscribe("/pose_traj", 1,
            &IKControlNode::pose_traj_cb, this);

    mm_joint_states_sub = nh.subscribe("/mm_joint_states", 1,
            &IKControlNode::mm_joint_states_cb, this);

    ur10_joint_vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1);
    rb_joint_vel_pub = nh.advertise<geometry_msgs::Twist>("/ridgeback_velocity_controller/cmd_vel", 1);

    controller.init(nh, LINEAR_GAIN, ROTATIONAL_GAIN);

    q_act = JointVector::Zero();
    dq_act = JointVector::Zero();

    pose_received = false;
    traj_active = false;
}

// Control loop.
void IKControlNode::loop(const double hz) {
    ros::Rate rate(hz);

    // ROS_INFO("Control loop started, waiting for trajectory...");
    //
    // // Wait until we have a pose command to send any commands
    // while (ros::ok() && !traj_active) {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    //
    // ROS_INFO("First pose command received.");
    //
    // controller.tick();
    // rate.sleep();
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
        Vector3d pos_des;
        Vector3d v_ff;
        double now = ros::Time::now().toSec();

        if (!trajectory.sample(now, pos_des, v_ff, quat_des)) {
            ROS_INFO("Trajectory ended.");
            traj_active = false;
            continue;
        }

        // // Send zero velocity command if we fall outside interpolation range.
        // if (!lerp.sample(now, pos_des, v_ff)) {
        //     ROS_WARN("outside of sampling window");
        //     publish_joint_speeds(JointVector::Zero());
        //     continue;
        // }
        //
        // Quaterniond quat_des;
        // if (!slerp.sample(now, quat_des)) {
        //     ROS_WARN("outside of sampling window");
        //     publish_joint_speeds(JointVector::Zero());
        //     continue;
        // }

        JointVector dq_cmd;
        bool success = controller.update(pos_des, quat_des, v_ff, w_ff, q_act, dq_cmd);
        if (!success) {
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


void IKControlNode::publish_joint_speeds(const JointVector& dq_cmd) {
    // Split into base and arm joints to send out.
    Vector3d dq_cmd_rb = dq_cmd.topRows<3>();
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


// We do interpolation whenever a new point comes in.
void IKControlNode::pose_cmd_cb(const mm_msgs::PoseTrajectoryPoint& msg) {
    // Desired linear position and velocity.
    Vector3d pos_des, v_des;
    pos_des << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    v_des << msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z;

    // Desired rotation and angular velocity.
    quat_des = Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x,
                           msg.pose.orientation.y, msg.pose.orientation.z);
    w_ff(0) = msg.velocity.angular.x;
    w_ff(1) = msg.velocity.angular.y;
    w_ff(2) = msg.velocity.angular.z;

    // Actual pose.
    Vector3d pos_act = w_T_e_act.translation();
    Quaterniond quat_act = Quaterniond(w_T_e_act.rotation());
    Vector3d v_act = dw_T_e_act.topRows<3>();
    Vector3d w_act = dw_T_e_act.bottomRows<3>();

    // Time
    double time_from_start = msg.time_from_start.toSec();
    double now = ros::Time::now().toSec();

    double t1 = now;
    double t2 = now + time_from_start;

    // Interpolate the trajectory, from which we sample later.
    lerp.interpolate(t1, t2, pos_act, pos_des, v_act, v_des);
    slerp.interpolate(t1, t2, quat_act, quat_des);

    pose_received = true;
}


// An entire trajectory is sent
// Single point method is a special case
void IKControlNode::pose_traj_cb(const mm_msgs::PoseTrajectory& msg) {
    trajectory.init(msg, 0.1); // TODO hardcoded for now
    traj_active = true;
    ROS_INFO("Trajectory started.");
}


void IKControlNode::mm_joint_states_cb(const sensor_msgs::JointState& msg) {
    for (int i = 0; i < mm::NUM_JOINTS; ++i) {
        q_act(i) = msg.position[i];
        dq_act(i) = msg.velocity[i];
    }
    update_forward_kinematics();
}


void IKControlNode::update_forward_kinematics() {
    Kinematics::calc_w_T_b(q_act, w_T_b_act);
    Kinematics::calc_w_T_e(q_act, w_T_e_act);

    Kinematics::forward_vel(q_act, dq_act, dw_T_e_act);
}

} // namespace mm

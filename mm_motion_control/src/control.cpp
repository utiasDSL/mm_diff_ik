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
Matrix3d ROTATIONAL_GAIN = Matrix3d::Identity();


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


void pose_msg_from_eigen(const Eigen::Affine3d& T, geometry_msgs::Pose& msg) {
    Eigen::Vector3d p = T.translation();
    Eigen::Quaterniond q(T.rotation());
    pose_msg_from_eigen(p, q, msg);
}


bool IKController::init(Matrix3d& Kv, Matrix3d& Kw) {
    this->Kv = Kv;
    this->Kw = Kw;
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

    // ROS_INFO_STREAM("x = " << quat_act.x() << "\n" <<
    //                 "y = " << quat_act.y() << "\n" <<
    //                 "z = " << quat_act.z() << "\n" <<
    //                 "w = " << quat_act.w());

    Eigen::Quaterniond quat_err = quat_des * quat_act.inverse();

    // Keep quaternion positive. Otherwise, we can get in a situation where it
    // flips between the negative and positive versions and doesn't go
    // anywhere.
    if (quat_err.w() < 0) {
        quat_err.coeffs() *= -1;
    }
    Vector3d rot_err;
    rot_err << quat_err.x(), quat_err.y(), quat_err.z();

    // Velocity command in task space: P control with velocity feedforward.
    // At the moment we assume zero rotational feedforward.
    Vector3d v = Kv * pos_err + v_ff;
    Vector3d w = Kw * rot_err + w_ff;

    // TODO I want this to be included in the published message as well
    // ROS_INFO_STREAM("angle_err = " << rot_err);
    ROS_INFO_STREAM(rot_err);

    Vector6d vel_cmd;
    vel_cmd << v, w;

    // Update time.
    double now = ros::Time::now().toSec();
    double dt = now - time_prev;
    time_prev = now;

    // Optimize to solve IK problem.
    return optimizer.solve(q_act, vel_cmd, dt, dq_cmd);
}


bool IKControlNode::init(ros::NodeHandle& nh) {
    // TODO later we probably want to allow multiple pose waypoints for
    // when we're not doing force servoing
    pose_cmd_sub = nh.subscribe("/pose_cmd", 1,
            &IKControlNode::pose_cmd_cb, this);

    mm_joint_states_sub = nh.subscribe("/mm_joint_states", 1,
            &IKControlNode::mm_joint_states_cb, this);

    ur10_joint_vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1);
    rb_joint_vel_pub = nh.advertise<geometry_msgs::Twist>("/ridgeback_velocity_controller/cmd_vel", 1);

    mm_pose_pub = new realtime_tools::RealtimePublisher<mm_msgs::PoseControlState>(nh, "/mm_pose_state", 1);

    q_act = JointVector::Zero();
    dq_act = JointVector::Zero();

    pose_received = false;
}

// Control loop.
void IKControlNode::loop(const double hz) {
    ros::Rate rate(hz);

    ROS_INFO("Control loop started, waiting for pose command...");

    // Wait until we have a pose command to send any commands
    while (ros::ok() && !pose_received) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("First pose command received.");

    // The controller is initialized here so that the time step of the
    // first iteration is not dependent on how long it takes to receive a
    // pose command. Then we sleep to have a typical timestep.
    controller.init(LINEAR_GAIN, ROTATIONAL_GAIN);
    rate.sleep();

    while (ros::ok()) {
        ros::spinOnce();

        // Sample interpolated trajectory.
        Vector3d pos_des;
        Vector3d v_ff;
        double now = ros::Time::now().toSec();

        // Send zero velocity command if we fall outside interpolation range.
        if (!trajectory.sample(now, pos_des, v_ff)) {
            ROS_WARN("outside of sampling window");
            publish_joint_speeds(JointVector::Zero());
            continue;
        }

        Quaterniond quat_des;
        if (!slerp.sample(now, quat_des)) {
            ROS_WARN("outside of sampling window");
            publish_joint_speeds(JointVector::Zero());
            continue;
        }

        JointVector dq_cmd;
        bool success = controller.update(pos_des, quat_des, v_ff, w_ff, q_act, dq_cmd);
        if (!success) {
            // Send zero velocity command if optimization fails (the velocity
            // commands are garbage in this case).
            ROS_WARN("Optimization failed");
            publish_joint_speeds(JointVector::Zero());
            continue;
        }

        publish_joint_speeds(dq_cmd);
        publish_mm_pose_state(pos_des, quat_des);

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


void IKControlNode::publish_mm_pose_state(const Eigen::Vector3d& pos_des,
                                          const Eigen::Quaterniond& quat_des) {
    geometry_msgs::Pose w_T_e_act_msg, w_T_e_des_msg;

    pose_msg_from_eigen(w_T_e_act, w_T_e_act_msg);
    pose_msg_from_eigen(pos_des, quat_des, w_T_e_des_msg);

    // TODO not sure if this lock breaks the single threaded assumptions I have
    // elsewhere
    if (mm_pose_pub->trylock()) {
        mm_pose_pub->msg_.actual.clear();
        mm_pose_pub->msg_.desired.clear();

        mm_pose_pub->msg_.actual.push_back(w_T_e_act_msg);
        mm_pose_pub->msg_.desired.push_back(w_T_e_des_msg);

        mm_pose_pub->msg_.header.frame_id = "world";
        mm_pose_pub->msg_.header.stamp = ros::Time::now();
        mm_pose_pub->unlockAndPublish();
    }
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
    trajectory.interpolate(t1, t2, pos_act, pos_des, v_act, v_des);
    slerp.interpolate(t1, t2, quat_act, quat_des);

    pose_received = true;
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

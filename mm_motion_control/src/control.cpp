#include <Eigen/Eigen>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <mm_msgs/PoseTrajectoryPoint.h>
#include <mm_msgs/PoseTrajectory.h>
#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/optimize.h"
#include "mm_motion_control/interp.h"
#include "mm_motion_control/control.h"


using namespace Eigen;


namespace mm {

// Controller gain.
Matrix3d P_GAIN = Matrix3d::Identity();

bool IKController::init(Matrix3d& K) {
    this->K = K;
    time_prev = ros::Time::now().toSec();
}

bool IKController::update(const Vector3d& pos_des,
                          const tf::Quaternion& rot_des,
                          const Vector3d& vel_ff,
                          const JointVector& q_act, JointVector& dq_cmd) {
    // Calculate actual pose using forward kinematics.
    Affine3d ee_pose_act;
    Kinematics::forward(q_act, ee_pose_act);

    // Position error.
    Vector3d pos_act = ee_pose_act.translation();
    Vector3d pos_err = pos_des - pos_act;

    // Orientation error.
    Eigen::Quaterniond q(ee_pose_act.rotation());
    tf::Quaternion rot_act(q.x(), q.y(), q.z(), q.w());
    tf::Quaternion quat_err = rot_act.inverse() * rot_des;
    tf::Vector3 rot_err = quat_err.getAxis() * quat_err.getAngle(); // TODO does this do what I think it does?
    // TODO we can also do P control on orientation error now

    // Velocity command in task space: P control with velocity
    // feedforward.
    Vector3d linear_vel = K * pos_err + vel_ff;

    Vector6d vel_cmd;
    vel_cmd << linear_vel, Vector3d::Zero();

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

    q_act = JointVector::Zero();
    dq_act = JointVector::Zero();

    pose_received = false;
}

// Control loop.
void IKControlNode::loop(const double hz) {
    ros::Rate rate(hz);

    ROS_INFO("Control loop started");

    // Wait until we have a pose command to send any commands
    while (ros::ok() && !pose_received) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("First pose command received.");

    // The controller is initialized here so that the time step of the
    // first iteration is not dependent on how long it takes to receive a
    // pose command. Then we sleep to have a typical timestep.
    controller.init(P_GAIN);
    rate.sleep();

    while (ros::ok()) {
        // service any callbacks
        ros::spinOnce();

        // Sample interpolated trajectory.
        Vector3d pos_des;
        Vector3d vel_ff;
        double now = ros::Time::now().toSec();

        // If we fall outside the interpolation range, do not send any
        // commands.
        // TODO need to validate robot behaviour in this case - will it
        // stop (as desired), or somehow keep moving based on previous
        // velocity?
        if (!trajectory.sample(now, pos_des, vel_ff)) {
            ROS_INFO("outside of sampling window");
            continue;
        }

        tf::Quaternion q = tf::Quaternion::getIdentity();

        JointVector dq_cmd;
        bool success = controller.update(pos_des, q, vel_ff, q_act, dq_cmd);
        if (!success) {
            // If the optimization fails we don't want to send commands to
            // the robot.
            ROS_INFO("Optimization failed");
            continue;
        }

        // Split into base and arm joints to send out.
        Vector3d dq_cmd_rb = dq_cmd.topRows<3>();
        Vector6d dq_cmd_ur10 = dq_cmd.bottomRows<6>();

        // Convert to JointTrajectory message with a single point (i.e.
        // velocity servoing) to publish to UR10.
        trajectory_msgs::JointTrajectoryPoint point;
        point.velocities = std::vector<double>(
                dq_cmd_ur10.data(), dq_cmd_ur10.data() + dq_cmd_ur10.size());
        trajectory_msgs::JointTrajectory traj;
        traj.points.push_back(point);

        ur10_joint_vel_pub.publish(traj);

        // Publish to base.
        geometry_msgs::Twist twist_rb;
        twist_rb.linear.x = dq_cmd_rb(0);
        twist_rb.linear.y = dq_cmd_rb(1);
        twist_rb.angular.z = dq_cmd_rb(2);

        rb_joint_vel_pub.publish(twist_rb);

        rate.sleep();
    }
}


// We do interpolation whenever a new point comes in.
void IKControlNode::pose_cmd_cb(const mm_msgs::PoseTrajectoryPoint& msg) {
    Vector3d pos;
    pos << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

    Vector3d vel;
    vel << msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z;

    tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y,
                     msg.pose.orientation.z, msg.pose.orientation.w);

    double time_from_start = msg.time_from_start.toSec();
    double now = ros::Time::now().toSec();

    double t1 = now;
    double t2 = now + time_from_start;

    // Only care about position for now.
    Vector3d pos_act = w_T_e_act.translation();
    Vector3d vel_act = dw_T_e_act.block<3,1>(0,0);

    // Interpolate the trajectory, from which we sample later.
    trajectory.interpolate(t1, t2, pos_act, pos, vel_act, vel);

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
    // TODO we could actually just store the Jacobian for reuse later,
    // since we need it both for control and interpolation.
    Kinematics::forward(q_act, w_T_e_act);
    Kinematics::forward_vel(q_act, dq_act, dw_T_e_act);
}

} // namespace mm

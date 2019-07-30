#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <mm_msgs/PoseTrajectoryPoint.h>
#include <mm_msgs/PoseTrajectory.h>

#include "mm/mm.h"
#include "mm/kinematics.h"
#include "mm/optimize.h"
#include "mm/interp.h"


using namespace Eigen;


namespace mm {
    class IKController {
        public:
            IKController() : optimizer() {}

            bool init(Matrix3d& K) {
                this->K = K;
            }

            // Run one iteration of the control loop.
            // pose_des: desired pose
            // vel_ff:   feedforward pose velocity
            // q_act:    current value of joint angles
            // dq_cmd:   populated with joint velocity commands to send
            void update(const Vector3d& pos_des, const Vector3d& vel_ff,
                        const QVector& q_act, QVector& dq_cmd);

        private:
            // TODO may want to store previous time for later use in the
            // optimizer

            // Proportional gain on the end effector pose error (in task space);
            Matrix3d K;

            // Optimizer to solve for joint velocity commands to send to the
            // robot.
            IKOptimizer optimizer;
    };


    void IKController::update(const Vector3d& pos_des, const Vector3d& vel_ff,
                      const QVector& q_act, QVector& dq_cmd) {
        // Calculate actual pose using forward kinematics.
        Affine3d ee_pose_act;
        Kinematics::forward(q_act, ee_pose_act);

        Vector3d pos_act = ee_pose_act.translation();
        Vector3d pos_err = pos_des - pos_act;

        // Velocity command in task space: P control with velocity
        // feedforward.
        Vector3d linear_vel = K * pos_err + vel_ff;

        Vector6d vel_cmd;
        vel_cmd << linear_vel, Vector3d::Zero();

        // Optimize to solve IK problem.
        bool success = optimizer.solve(q_act, vel_cmd, dq_cmd);
        if (!success) {
            ROS_INFO("Optimization failed");
        }
    }


    class IKControlNode {
        public:
            IKControlNode() : controller() {}

            bool init(ros::NodeHandle& nh);

            // Control loop.
            void loop(const double hz);

        private:
            /** TYPEDEFS **/

            // TODO I'm not sure if this is appropriate for publishing actual
            // control commands, which I don't particularly wish to be
            // throttled
            // typedef realtime_tools::RealtimePublisher<trajectory_msgs::JointTrajectory> JointVelocityPublisher;


            /** VARIABLES **/

            // Subsribe to desired end effector pose.
            ros::Subscriber pose_cmd_sub;

            // Subscribe to current joint values of the robot.
            ros::Subscriber ur10_joint_states_sub;

            // Get (x, y, theta) from Vicon
            ros::Subscriber rb_state_sub;

            // Publishers for desired joint speeds calculated by controller.
            ros::Publisher ur10_joint_vel_pub;
            ros::Publisher rb_joint_vel_pub;

            IKController controller;

            // Actual joint positions, updated by the subscriber
            QVector q_act;
            QVector dq_act;

            // Actual pose and twist of end effector.
            Affine3d w_T_e_act;
            Vector6d dw_T_e_act;

            // Cubic polynomial trajectory, interpolated between current state
            // and commanded pose.
            // TODO we need sane defaults for this
            CubicInterp<3> trajectory;

            // Set to true once a pose command has been received. Before that,
            // we don't want to send any commands.
            bool pose_received;


            /** FUNCTIONS **/

            // Takes a single Pose trajectory point to control toward.
            void pose_cmd_cb(const mm_msgs::PoseTrajectoryPoint& msg);

            // Update state of UR10 joints.
            void ur10_joint_states_cb(const sensor_msgs::JointState& msg);

            // Update state of Ridgeback joints
            // TODO this is going to change once we add Vicon, which has a
            // different API.
            void rb_joint_states_cb(const sensor_msgs::JointState& msg);

            void update_forward_kinematics();
    };


    bool IKControlNode::init(ros::NodeHandle& nh) {
        // TODO later we probably want to allow multiple pose waypoints for
        // when we're not doing force servoing
        pose_cmd_sub = nh.subscribe("/pose_cmd", 1,
                &IKControlNode::pose_cmd_cb, this);

        ur10_joint_states_sub = nh.subscribe("/ur10_joint_states", 1,
                &IKControlNode::ur10_joint_states_cb, this);

        // TODO need vicon for this
        // rb_state_sub

        ur10_joint_vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1);

        rb_joint_vel_pub = nh.advertise<geometry_msgs::Twist>("/ridgeback_velocity_controller/cmd_vel", 1);

        // TODO probably want to publish some sort of state here

        // Initialize the controller.
        Matrix3d K = Matrix3d::Identity();
        controller.init(K);

        q_act = QVector::Zero();
        dq_act = QVector::Zero();

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
                break;
            }
            // ROS_INFO_STREAM("pos_des = " << pos_des << "\nvel_ff = " << vel_ff);

            QVector dq_cmd;
            controller.update(pos_des, vel_ff, q_act, dq_cmd);

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
            twist_rb.linear.z = dq_cmd_rb(2);

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
        ROS_INFO("new pose cmd received");
    }


    void IKControlNode::rb_joint_states_cb(const sensor_msgs::JointState& msg) {
        // Update current joint states.
        for (int i = 0; i < 3; ++i) {
            q_act(i)  = msg.position[i];
            dq_act(i) = msg.velocity[i];
        }
        update_forward_kinematics();
    }


    void IKControlNode::ur10_joint_states_cb(const sensor_msgs::JointState& msg) {
        // order is [ur10_arm_shoulder_pan_joint, ur10_arm_shoulder_lift_joint,
        // ur10_arm_elbow_joint, ur10_arm_wrist_1_joint,
        // ur10_arm_wrist_2_joint, ur10_arm_wrist_3_joint]

        // Reinitialize current joint states.
        for (int i = 3; i < 9; ++i) {
            q_act(i)  = msg.position[i];
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
}

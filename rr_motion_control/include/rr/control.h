#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <rr_msgs/PoseTrajectoryPoint.h>
#include <rr_msgs/PoseTrajectory.h>

#include "rr/rr.h"
#include "rr/kinematics.h"
#include "rr/optimize.h"
#include "rr/interp.h"


using namespace Eigen;


namespace rr {
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
                optimizer.solve(q_act, vel_cmd, dq_cmd);
            }

        private:
            // TODO may want to store previous time for later use in the
            // optimizer

            // Proportional gain on the end effector pose error (in task space);
            Matrix3d K;

            // Optimizer to solve for joint velocity commands to send to the
            // robot.
            IKOptimizer optimizer;
    };


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

            // Publisher for desired joint speeds calculated by controller.
            // JointVelocityPublisher joint_vel_pub;
            ros::Publisher joint_vel_pub;

            IKController controller;

            // All of these are only written by the subs and read everywhere
            // else, so *should* be pretty thread-safe - but we're also not
            // multi-threaded at the moment.

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


            /** FUNCTIONS **/

            // Takes a single Pose trajectory point to control toward.
            void pose_cmd_cb(const rr_msgs::PoseTrajectoryPoint& msg);

            void ur10_joint_states_cb(const sensor_msgs::JointState& msg);

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

        joint_vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1);

        // TODO probably want to publish some sort of state here

        // Initialize the controller.
        Matrix3d K = Matrix3d::Identity();
        controller.init(K);

        q_act = QVector::Zero();
        dq_act = QVector::Zero();
    }

    // Control loop.
    void IKControlNode::loop(const double hz) {
        ros::Rate rate(hz);

        while (ros::ok()) {
            // service any callbacks
            ros::spinOnce();

            Vector3d pos_act = w_T_e_act.translation();
            Vector3d vel_act = dw_T_e_act.block<3,1>(0,0);

            QVector dq_cmd;
            controller.update(pos_act, vel_act, q_act, dq_cmd);

            // Convert to JointTrajectory message with a single point (i.e.
            // velocity servoing)
            trajectory_msgs::JointTrajectoryPoint point;
            point.velocities = std::vector<double>(dq_cmd.data(), dq_cmd.data() + dq_cmd.size());
            trajectory_msgs::JointTrajectory traj;
            traj.points.push_back(point);

            joint_vel_pub.publish(traj);

            rate.sleep();
        }
    }


    // We do interpolation whenever a new point comes in.
    void IKControlNode::pose_cmd_cb(const rr_msgs::PoseTrajectoryPoint& msg) {
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
    }


    void IKControlNode::ur10_joint_states_cb(const sensor_msgs::JointState& msg) {
        // order is [ur10_arm_shoulder_pan_joint, ur10_arm_shoulder_lift_joint,
        // ur10_arm_elbow_joint, ur10_arm_wrist_1_joint,
        // ur10_arm_wrist_2_joint, ur10_arm_wrist_3_joint]

        // Reinitialize current joint states.
        for (int i = 0; i < 6; ++i) {
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

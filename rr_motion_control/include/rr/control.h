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


using namespace Eigen;


namespace rr {
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
            ros::Subscriber joint_states_sub;

            // Publisher for desired joint speeds calculated by controller.
            // JointVelocityPublisher joint_vel_pub;
            ros::Publisher joint_vel_pub;

            IKController controller;

            // All of these are only written by the subs and read everywhere
            // else, so *should* be pretty thread-safe - but we're also not
            // multi-threaded at the moment.

            // Affine3d pose_des;
            // Vector6d
            // TODO this needs to be interpolated each time a new cmd comes in
            trajectory_interface::QuinticSplineSegment<double> spline;

            // Actual joint positions, updated by the subscriber
            QVector q_act;


            /** FUNCTIONS **/

            // Takes a single Pose trajectory point to control toward.
            void pose_cmd_cb(const rr_msgs::PoseTrajectoryPoint& msg);

            void joint_states_cb(const sensor_msgs::JointState& msg);
    };

    class IKController {
        public:
            IKController(Matrix6d& K) : K(K), optimizer() {}

            // Run one iteration of the control loop.
            // pose_des: desired pose
            // vel_ff:   feedforward pose velocity
            // q_act:    current value of joint angles
            // dq_cmd:   populated with joint velocity commands to send
            void update(const Affine3d& pose_des, const Vector6d& vel_ff,
                        const QVector& q_act, QVector& dq_cmd) {

                // Calculate actual pose using forward kinematics.
                Affine3d pose_act;
                Kinematics::forward(q_act, pose_act);

                Vector3d pos_des = pose_des.translation();
                Vector3d pos_act = pose_act.translation();
                Vector3d pos_err = pos_des - pos_act;

                Matrix3d K_pos = K.block<3,3>(0,0);

                // Velocity command in task space.
                Vector3d vel_cmd = K_pos * pos_err + vel_ff;

                // TODO need to determine interface here
                optimizer.solve()
            }

        private:
            // Time step between control loop iterations.
            double dt;

            // Proportional gain on the end effector pose error (in task space);
            Matrix6d K;

            // Optimizer to solve for joint velocity commands to send to the
            // robot.
            IKOptimizer optimizer;
    };


    bool IKControlNode::init(ros::NodeHandle& nh) {
        pose_cmd_sub = nh.subscribe("/pose_cmd", 1,
                &IKControlNode::pose_cmd_sub, this);

        // TODO probably need to subscribe to vicon to get correct value for
        // base joints
        joint_states_sub = nh.subscribe("/joint_states", 1,
                &IKControlNode::joint_states_sub, this);

        // TODO need to figure out the API for this
        joint_vel_pub = np.advertise<>("", 1);

    }

    // Control loop.
    void IKControlNode::loop(const double hz) {
        ros::Rate rate(hz);

        while (ros::ok()) {
            ros::spinOnce();

            QVector dq_cmd;
            controller.update(,dq_cmd);

            // TODO
            joint_vel_pub.publish();

            rate.sleep()
        }
    }

    void IKControlNode::pose_cmd_cb(const rr_msgs::PoseTrajectoryPoint& msg) {
        // do interpolation
        // store trajectory somehow
    }


    void IKControlNode::joint_states_cb(const sensor_msgs::JointState& msg) {
        //std::vector<std::string>::iterator it = std::find(msg.name.begin(), msg.name.end, "name");
        // extract joint information from message
    }
}

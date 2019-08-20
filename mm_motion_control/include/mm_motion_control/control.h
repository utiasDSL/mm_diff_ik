#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
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


using namespace Eigen;


namespace mm {


class IKController {
    public:
        IKController() : optimizer() {}

        bool init(Matrix3d& K);

        // Run one iteration of the control loop.
        //
        // pose_des: desired pose
        // vel_ff:   feedforward pose velocity
        // q_act:    current value of joint angles
        // dq_cmd:   populated with joint velocity commands to send
        //
        // Returns true if the optimization problem was solved sucessfully,
        // false otherwise.
        bool update(const Vector3d& pos_des, const tf::Quaternion& rot_des,
                    const Vector3d& vel_ff, const JointVector& q_act,
                    JointVector& dq_cmd);

    private:
        // Time from previous control loop iteration.
        double time_prev;

        // Proportional gain on the end effector pose error (in task space);
        Matrix3d K;

        // Optimizer to solve for joint velocity commands to send to the
        // robot.
        IKOptimizer optimizer;
}; // class IKController


class IKControlNode {
    public:
        IKControlNode() : controller() {}

        bool init(ros::NodeHandle& nh);

        // Control loop.
        void loop(const double hz);

    private:
        /** VARIABLES **/

        // Subsribe to desired end effector pose.
        ros::Subscriber pose_cmd_sub;

        // Subscribe to current joint values of the robot.
        ros::Subscriber mm_joint_states_sub;

        // Publishers for desired joint speeds calculated by controller.
        ros::Publisher ur10_joint_vel_pub;
        ros::Publisher rb_joint_vel_pub;

        IKController controller;

        // Actual joint positions, updated by the subscriber
        JointVector q_act;
        JointVector dq_act;

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

        // Update state of robot joints.
        void mm_joint_states_cb(const sensor_msgs::JointState& msg);

        void update_forward_kinematics();
}; // class IKControlNode

} // namespace mm

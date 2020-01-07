#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <mm_msgs/PoseTrajectory.h>
#include <mm_msgs/PoseControlState.h>
#include <mm_msgs/Obstacles.h>
#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/optimize.h"
#include "mm_motion_control/trajectory.h"
#include "mm_motion_control/obstacle.h"

#include <geometry_msgs/Vector3.h>


namespace mm {


class IKController {
    public:
        IKController() : optimizer() {}

        bool init(ros::NodeHandle& nh, Eigen::Matrix3d& Kv, Eigen::Matrix3d& Kw);

        // Run one iteration of the control loop.
        //
        // pose_des: desired pose
        // vel_ff:   feedforward pose velocity
        // q_act:    current value of joint angles
        // dq_cmd:   populated with joint velocity commands to send
        //
        // Returns 0 if the optimization problem was solved sucessfully,
        // otherwise a non-zero status code.
        int update(const Eigen::Vector3d& pos_des,
                   const Eigen::Quaterniond& quat_des,
                   const Eigen::Vector3d& v_ff,
                   const Eigen::Vector3d& w_ff,
                   const JointVector& q_act,
                   const JointVector& dq_act,
                   const std::vector<ObstacleModel> obstacles,
                   JointVector& dq_cmd);

        // Reset the stored previous time to current time.
        void tick();

    private:
        // Pattern followed by ROS control for typedefing.
        typedef realtime_tools::RealtimePublisher<mm_msgs::PoseControlState> StatePublisher;
        typedef std::unique_ptr<StatePublisher> StatePublisherPtr;

        // Time from previous control loop iteration.
        double time_prev;

        // Proportional gains on the end effector pose error (in task space),
        // for linear and rotational error, respectively.
        Eigen::Matrix3d Kv;
        Eigen::Matrix3d Kw;

        // Optimizer to solve for joint velocity commands to send to the
        // robot.
        IKOptimizer optimizer;

        StatePublisherPtr state_pub;

        // Publish state of the end effector.
        void publish_state(const ros::Time& time,
                           const Eigen::Vector3d&    pos_act,
                           const Eigen::Quaterniond& quat_act,
                           const Eigen::Vector3d&    pos_des,
                           const Eigen::Quaterniond& quat_des,
                           const Eigen::Vector3d&    pos_err,
                           const Eigen::Quaterniond& quat_err,
                           const Eigen::Vector3d&    vel_des);
}; // class IKController


class IKControlNode {
    public:
        IKControlNode() : controller() {}

        bool init(ros::NodeHandle& nh);

        // Control loop.
        void loop(const double hz);

    private:
        /** VARIABLES **/

        // Subsribe to desired end effector pose trajectories.
        ros::Subscriber pose_traj_sub;

        ros::Subscriber point_traj_sub;

        // Subscribe to current joint values of the robot.
        ros::Subscriber mm_joint_states_sub;

        // Subscribe to position offset generated by force control.
        ros::Subscriber force_position_offset_sub;

        // Subscribe to obstacle detections.
        ros::Subscriber obstacle_sub;

        // Publishers for desired joint speeds calculated by controller.
        ros::Publisher ur10_joint_vel_pub;
        ros::Publisher rb_joint_vel_pub;

        IKController controller;

        // Actual joint positions, updated by the subscriber
        JointVector q_act;
        JointVector dq_act;

        // Actual pose and twist of end effector.
        Eigen::Affine3d w_T_e_act;
        Vector6d dw_T_e_act;

        // Base pose.
        Eigen::Affine3d w_T_b_act;

        // Position offset from the force controller.
        Eigen::Vector3d pos_offset;

        // Trajectory interpolator.
        PoseTrajectory trajectory;

        std::vector<ObstacleModel> obstacles;

        // True if we currently have a trajectory to follow, false otherwise.
        bool traj_active;


        /** FUNCTIONS **/

        // Receive a trajectory of waypoints to follow.
        void pose_traj_cb(const mm_msgs::PoseTrajectory& msg);

        // Receive a command to keep the EE in place (but the motion control
        // loop is still running so it can respond to e.g. applied forces).
        void point_traj_cb(const geometry_msgs::PoseStamped& msg);

        // Update state of robot joints.
        void mm_joint_states_cb(const sensor_msgs::JointState& msg);

        // Receive position offset generated by force control.
        void pos_offset_cb(const geometry_msgs::Vector3Stamped& msg);

        // Receive list of obstacles for the base to avoid.
        void obstacle_cb(const mm_msgs::Obstacles& msg);

        // Calculate poses in task space based on current joint state.
        void update_forward_kinematics();

        // Publish joint speeds computed by the controller to the arm and base.
        void publish_joint_speeds(const JointVector& dq_cmd);
}; // class IKControlNode

} // namespace mm

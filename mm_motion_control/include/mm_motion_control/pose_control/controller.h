#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>

#include <mm_kinematics/kinematics.h>
#include <mm_msgs/PoseTrajectory.h>
#include <mm_msgs/PoseControlState.h>
#include <mm_msgs/Obstacles.h>

#include "mm_motion_control/pose_control/optimizer.h"
#include "mm_motion_control/pose_control/trajectory.h"
#include "mm_motion_control/pose_control/obstacle.h"


namespace mm {


// The IKController wraps the IKOptimizer to perform additional preprocessing
// and publishing.
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
        // TODO I actually want to access this as a method and publish from the
        // manager
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

} // namespace mm

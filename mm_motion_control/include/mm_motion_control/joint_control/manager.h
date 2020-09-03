#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <mm_kinematics/kinematics.h>

#include "mm_motion_control/joint_control/controller.h"


namespace mm {

// Proportional gain matrix.
const static JointMatrix K = 0.5*JointMatrix::Identity();

// Maximum joint speed.
const static double MAX_DQ = 0.2;

// Default home positions -- this can also be set by parameter.
const static JointVector DEFAULT_HOME((JointVector()
            << 0.0, 0.0, 0.0,
               0.0, -0.75*M_PI, -M_PI_2, -0.75*M_PI, -M_PI_2, M_PI_2).finished());

// Lower EE position; good for pushing.
// const static JointVector HOME((JointVector()
//             << 0.0, 0.0, 0.0,
//                0.0, -2.70526, -M_PI_2, -2.007128, -M_PI_2, M_PI_2).finished());


// Manager for joint-space controllers.
class JointControllerManager {
    public:
        JointControllerManager() : controller() {}

        bool init(ros::NodeHandle& nh);

        // Enter control loop.
        // Parameters:
        //   hz: rate at which to run the loop
        void loop(const double hz);

    private:
        /* VARIABLES */

        JointController controller;

        // Subscriber for current joint state of the robot.
        ros::Subscriber mm_joint_states_sub;

        // Subscriber for joint position commands.
        ros::Subscriber mm_joint_cmd_sub;

        // Publishers for joint speed commands.
        ros::Publisher ur10_joint_vel_pub;
        ros::Publisher rb_joint_vel_pub;

        JointVector q_act; // Actual joint positions.
        JointVector q_des; // Desired joint positions.

        bool joint_state_rec;

        /* FUNCTIONS */

        // Publish commanded joint speeds to the robot.
        void publish_joint_speeds(const JointVector& dq_cmd);

        // Callback to update joint positions and velocities from the robot.
        void mm_joint_states_cb(const sensor_msgs::JointState& msg);

}; // class JointControllerManager

}

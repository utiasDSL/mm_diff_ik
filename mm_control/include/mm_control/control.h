#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <sensor_msgs/JointState.h>
#include <mm_kinematics/kinematics.h>


namespace mm {


// Base controller from which others must be derived.
class MMController {
    public:
        MMController();
        ~MMController();

        // Initialize the node: setup subscribers and publishers, etc.
        bool init(ros::NodeHandle& nh, const double hz);

        // Enter control loop.
        virtual void loop() = 0;

    protected:
        /* VARIABLES */

        // Set to true if an unsafe action is detected so the robot can be
        // stopped.
        bool did_become_unsafe;

        // Controller timestep and rate. dt = 1 / hz
        double dt, hz;

        // Subscriber for current joint state of the robot.
        ros::Subscriber mm_joint_states_sub;

        // Publishers for joint speed commands.
        ros::Publisher ur10_joint_vel_pub;
        ros::Publisher rb_joint_vel_pub;

        JointVector q;
        JointVector dq;
        JointVector u;

        // True if a joint state message has been received.
        bool joint_state_rec;

        // Time when the last joint state message was received.
        double last_joint_state_time;

        /* FUNCTIONS */

        // Callback to update joint positions and velocities from the robot.
        void mm_joint_states_cb(const sensor_msgs::JointState& msg);

        // Publish joint speed commands to the robot. Should be called from
        // loop().
        void publish_joint_speeds(const ros::Time& now);

        // Generate new control commands. Should be called from loop().
        virtual int update(const ros::Time& now) = 0;

        // Publish state of the controller, for logging and analysis. Should be
        // called from loop().
        virtual void publish_state(const ros::Time& now) = 0;

}; // class MMController

} // namespace mm

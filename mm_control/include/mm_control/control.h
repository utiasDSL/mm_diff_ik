#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <sensor_msgs/JointState.h>
#include <mm_kinematics/kinematics.h>


namespace mm {


// Base controller from which others must be derived.
class MMController {
    public:
        MMController() {}

        bool init(ros::NodeHandle& nh);

        // Enter control loop.
        // Parameters:
        //   hz: rate at which to run the loop
        virtual void loop(const double hz);

    protected:
        /* VARIABLES */

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

        // Publish joint speed commands to the robot.
        void publish_joint_speeds(const ros::Time& now);

        // Callback to update joint positions and velocities from the robot.
        void mm_joint_states_cb(const sensor_msgs::JointState& msg);

}; // class MMController

} // namespace mm

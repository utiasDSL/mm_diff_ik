#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>

#include <mm_math_util/filter.h>


namespace mm {

// The node listens to raw Vicon transform messages for the Ridgeback base and
// converts it into a JointState message which includes a numerically
// differentiated and filtered velocity estimate on (x, y, yaw).
class ViconEstimatorNode {
    public:
        ViconEstimatorNode() {}

        // Initialize the node.
        bool init(ros::NodeHandle& nh);

        // Enter ROS loop with given frequency in hertz.
        void loop(const double hz);

    private:
        /* VARIABLES */

        // Subscriber to Vicon pose.
        ros::Subscriber vicon_thing_base_sub;

        // Publisher for position and velocity of the base (x, y, theta).
        ros::Publisher rb_joint_states_pub;

        // Store last two poses for numerical differentiation.
        geometry_msgs::TransformStamped tf_prev;
        geometry_msgs::TransformStamped tf_curr;

        // Exponential smoothing filters to remove noise from numerically
        // differentiated velocity.
        ExponentialSmoother<double>   filter_rot_vel;
        ExponentialSmoother<Eigen::Vector2d> filter_lin_vel;

        // True after a new Vicon message has been received.
        bool new_msg;

        // Number of messages received.
        uint32_t msg_count;

        /* FUNCTIONS */

        void vicon_thing_base_cb(const geometry_msgs::TransformStamped& msg);

        void publish_joint_states();

}; // class ViconEstimatorNode

} // namespace mm

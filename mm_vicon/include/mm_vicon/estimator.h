#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>

#include "mm_vicon/filter.h"


using namespace Eigen;


namespace mm {
    class ViconEstimatorNode {
        public:
            ViconEstimatorNode() {}

            bool init(ros::NodeHandle& nh);

            void loop(const double hz);

        private:
            // Subscriber to Vicon pose.
            ros::Subscriber vicon_thing_base_sub;

            // Publisher for position and velocity of the base (x, y, theta).
            ros::Publisher rb_joint_states_pub;

            // Store last two poses for numerical differentiation.
            geometry_msgs::TransformStamped tf_prev;
            geometry_msgs::TransformStamped tf_curr;

            bool new_msg;

            void vicon_thing_base_cb(const geometry_msgs::TransformStamped& msg);

            void publish_joint_states();

    };
}

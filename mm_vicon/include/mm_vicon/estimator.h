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

            bool init(ros::NodeHandle& nh) {
                vicon_thing_base_sub = nh.subscribe(
                        "/vicon/ThingBase/ThingBase", 1,
                        &ViconEstimatorNode::vicon_thing_base_cb, this);

                rb_joint_states_pub = nh.advertise<sensor_msgs::JointState>(
                        "/rb_joint_states", 1);

                new_msg = false;
            }

            void loop(const double hz) {
                ros::Rate rate(hz);
                while (ros::ok()) {
                    // Fire callbacks.
                    ros::spinOnce();

                    if (new_msg) {
                        publish_joint_states();
                    }

                    rate.sleep();
                }
            }

        private:
            // Subscriber to Vicon pose.
            ros::Subscriber vicon_thing_base_sub;

            // Publisher for position and velocity of the base (x, y, theta).
            ros::Publisher rb_joint_states_pub;

            // Store last two poses for numerical differentiation.
            geometry_msgs::TransformStamped tf_prev;
            geometry_msgs::TransformStamped tf_curr;

            bool new_msg;

            void vicon_thing_base_cb(const geometry_msgs::TransformStamped& msg) {
                tf_prev = tf_curr;
                tf_curr = msg;

                new_msg = true;

                // double yaw  = tf::getYaw(msg.transform.rotation);
                // ROS_INFO_STREAM(yaw);
            }

            void publish_joint_states() {
                ros::Time t_prev = tf_prev.header.stamp;
                ros::Time t_curr = tf_curr.header.stamp;
                double dt = (t_curr - t_prev).toSec();

                double yaw_prev = tf::getYaw(tf_prev.transform.rotation);
                double yaw_curr = tf::getYaw(tf_curr.transform.rotation);

                Vector3d position_prev, position_curr;
                position_prev << tf_prev.transform.translation.x,
                                 tf_prev.transform.translation.y,
                                 tf_prev.transform.translation.z;
                position_curr << tf_curr.transform.translation.x,
                                 tf_curr.transform.translation.y,
                                 tf_curr.transform.translation.z;

                double vx = (position_curr(0) - position_prev(0)) / dt;
                double vy = (position_curr(1) - position_prev(1)) / dt;
                double vyaw = fmod(yaw_curr - yaw_prev, M_PI) / dt;

                sensor_msgs::JointState rb_joint_states;
                rb_joint_states.header.stamp = ros::Time::now();

                rb_joint_states.position.push_back(position_curr(0)); // x
                rb_joint_states.position.push_back(position_curr(1)); // y
                rb_joint_states.position.push_back(yaw_curr); // yaw

                rb_joint_states.velocity.push_back(vx);
                rb_joint_states.velocity.push_back(vy);
                rb_joint_states.velocity.push_back(vyaw);

                rb_joint_states_pub.publish(rb_joint_states);
            }

    };
}

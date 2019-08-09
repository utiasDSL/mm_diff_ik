#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

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
            }

            void loop(const double hz) {
                ros::Rate rate(hz);
                while (ros::ok()) {
                    // Fire callbacks.
                    ros::spinOnce();

                    ros::Time t_prev = tf_prev.header.stamp;
                    ros::Time t_curr = tf_curr.header.stamp;
                    double dt = (t_curr - t_prev).toSec();

                    Quaterniond quat_prev(tf_prev.transform.rotation.w,
                                          tf_prev.transform.rotation.x,
                                          tf_prev.transform.rotation.y,
                                          tf_prev.transform.rotation.z);
                    Vector3d rpy_prev = quat_prev.toRotationMatrix().eulerAngles(0, 1, 2);

                    Quaterniond quat_curr(tf_curr.transform.rotation.w,
                                          tf_curr.transform.rotation.x,
                                          tf_curr.transform.rotation.y,
                                          tf_curr.transform.rotation.z);
                    Vector3d rpy_curr = quat_curr.toRotationMatrix().eulerAngles(0, 1, 2);

                    // Difference between previous and current rotation.
                    Quaterniond dq = quat_prev.inverse() * quat_curr;
                    Vector3d drpy = quat_curr.toRotationMatrix().eulerAngles(0, 1, 2);

                    Vector3d position_prev, position_curr;

                    position_prev << tf_prev.transform.translation.x,
                                     tf_prev.transform.translation.y,
                                     tf_prev.transform.translation.z;

                    position_curr << tf_curr.transform.translation.x,
                                     tf_curr.transform.translation.y,
                                     tf_curr.transform.translation.z;

                    double vx = (position_curr(0) - position_prev(0)) / dt;
                    double vy = (position_curr(1) - position_prev(1)) / dt;
                    // double vyaw = fmod(rpy_curr(2) - rpy_prev(2), M_PI) / dt;
                    double vyaw = drpy(2) / dt;

                    sensor_msgs::JointState rb_joint_states;
                    rb_joint_states.header.stamp = ros::Time::now();

                    rb_joint_states.position.push_back(position_curr(0)); // x
                    rb_joint_states.position.push_back(position_curr(1)); // y
                    rb_joint_states.position.push_back(rpy_curr(2)); // yaw

                    rb_joint_states.velocity.push_back(vx);
                    rb_joint_states.velocity.push_back(vy);
                    rb_joint_states.velocity.push_back(vyaw);

                    rb_joint_states_pub.publish(rb_joint_states);

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

            void vicon_thing_base_cb(const geometry_msgs::TransformStamped& msg) {
                tf_prev = tf_curr;
                tf_curr = msg;
            }

    };
}

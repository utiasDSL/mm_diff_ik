#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>


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
                    ros::spinOnce();

                    ros::Time t_prev = pose_prev.header.stamp;
                    ros::Time t_curr = pose_curr.header.stamp;
                    double dt = (t_curr - t_prev).toSec();

                    Quaterniond quat_prev(pose_prev.pose.orientation.w,
                                          pose_prev.pose.orientation.x,
                                          pose_prev.pose.orientation.y,
                                          pose_prev.pose.orientation.z);
                    Vector3d rpy_prev = quat_prev.toRotationMatrix().eulerAngles(0, 1, 2);

                    Quaterniond quat_curr(pose_curr.pose.orientation.w,
                                          pose_curr.pose.orientation.x,
                                          pose_curr.pose.orientation.y,
                                          pose_curr.pose.orientation.z);
                    Vector3d rpy_curr = quat_curr.toRotationMatrix().eulerAngles(0, 1, 2);


                    Vector3d position_prev, position_curr;

                    position_prev << pose_prev.pose.position.x,
                                     pose_prev.pose.position.y,
                                     pose_prev.pose.position.z;

                    position_curr << pose_curr.pose.position.x,
                                     pose_curr.pose.position.y,
                                     pose_curr.pose.position.z;

                    double vx = (position_curr(0) - position_prev(0)) / dt;
                    double vy = (position_curr(1) - position_prev(1)) / dt;
                    double vyaw = fmod(rpy_curr(2) - rpy_prev(2), M_PI) / dt;

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

            // Store two poses for numerical differentiation.
            geometry_msgs::PoseStamped pose_prev;
            geometry_msgs::PoseStamped pose_curr;

            void vicon_thing_base_cb(const geometry_msgs::PoseStamped& msg) {
                pose_prev = pose_curr;
                pose_curr = msg;
            }

    };
}

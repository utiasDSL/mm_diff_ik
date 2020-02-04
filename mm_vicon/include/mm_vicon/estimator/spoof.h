#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>

#include <mm_math_util/filter.h>


namespace mm {


class ViconEstimatorNodeSim {
    public:
        ViconEstimatorNodeSim() {}

        bool init(ros::NodeHandle& nh);

        void loop(const double hz);

    private:
        ros::Subscriber rb_joint_vel_sub;
        ros::Publisher rb_joint_states_pub;

        double time_prev;
        Eigen::Vector3d q;  // Base joint positions.
        Eigen::Vector3d dq; // Base joint velocities.

        void rb_joint_vel_cb(const geometry_msgs::Twist& msg);

}; // class ViconEstimatorNodeSim

} // namespace mm

#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mm_msgs/ForceControlState.h>

#include <mm_kinematics/kinematics.h>
#include "mm_force_control/bias_estimator.h"


namespace mm {

class ForceControlNode {
    public:
        ForceControlNode(): {}

        bool init(ros::NodeHandle& nh) {

        }

        void loop(const double hz) {

        }

    private:
        ros::Subscriber mm_joint_states_sub;
        ros::Subscriber force_sub;

        ros::Publisher position_offset_pub;
        ros::Publisher state_pub;

}; // class ForceControlNode

} // namespace mm

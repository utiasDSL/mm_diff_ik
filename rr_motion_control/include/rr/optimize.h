#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include "rr/rr.h"


namespace rr {
    class IKOptimizer {
        // TODO optimize the hell out of the joint velocity
    };
}

#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mm_msgs/PoseTrajectory.h>

#include <mm_control/control.h>
#include <mm_motion_control/pose_control/trajectory.h>


namespace mm {

// Abstract base class for Cartesian controllers, which track task-space goals.
class CartesianController : public MMController {
    public:
        CartesianController() {}
        ~CartesianController() {}

        bool init(ros::NodeHandle& nh, const double hz);

    protected:
        /** VARIABLES **/

        ros::Publisher state_pub;

        // Subscribe to desired end effector pose trajectories.
        ros::Subscriber pose_traj_sub;
        ros::Subscriber point_traj_sub;

        // Trajectory interpolator.
        PoseTrajectory trajectory;

        // True if we currently have a trajectory to follow, false otherwise.
        bool traj_active;


        /** FUNCTIONS **/

        // Receive a trajectory of waypoints to follow.
        void pose_traj_cb(const mm_msgs::PoseTrajectory& msg);

        // Receive a command to keep the EE in place (but the motion control
        // loop is still running so it can respond to e.g. applied forces).
        void point_traj_cb(const geometry_msgs::PoseStamped& msg);

        void publish_state(const ros::Time& now);
};


// class MPController : CartesianController {
//
// };

} // namespace mm

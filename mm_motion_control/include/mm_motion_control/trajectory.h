#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <mm_msgs/PoseTrajectoryPoint.h>
#include <mm_msgs/PoseTrajectory.h>
#include <mm_msgs/conversions.h>
#include <geometry_msgs/Pose.h>
#include <mm_math_util/interp.h>


namespace mm {


class PoseTrajectory {
    public:
        PoseTrajectory() {}

        // Follow a trajectory of waypoints.
        bool follow(mm_msgs::PoseTrajectory trajectory) {
            t0 = ros::Time::now().toSec();
            dt = trajectory.dt.toSec();
            waypoints = trajectory.points;
            tf = t0 + (waypoints.size()-1) * dt;
            w_prev = Eigen::Vector3d::Zero();
            stationary = false;

            return true;
        }

        // Stay at a single pose.
        bool stay_at(const geometry_msgs::Pose pose) {
            mm_msgs::PoseTrajectoryPoint waypoint;
            waypoint.pose = pose;

            waypoints.clear();
            waypoints.push_back(waypoint);

            stationary = true;
        }

        // Sample the trajectory at time t.
        bool sample(const double t, Eigen::Vector3d& p, Eigen::Vector3d& v,
                    Eigen::Quaterniond& q, Eigen::Vector3d& w) {
            if (stationary) {
                pose_traj_point_to_eigen(waypoints.back(), p, q, v, w);
            } else {
                return interpolate(t, p, v, q, w);
            }
            return true;
        }

    private:
        double t0; // start time
        double tf; // end time
        double dt; // time step

        // List of waypoints defining the entire trajectory.
        std::vector<mm_msgs::PoseTrajectoryPoint> waypoints;

        // Cubic interpolation for the linear component.
        CubicInterp<3> lerp;

        // Spherical linear interpolation (slerp) for the angular component.
        QuaternionInterp slerp;

        // Assume angular velocity is constant throughout the
        // trajectory. Indeed, if it is not, then we should be doing
        // something more complicated than slerp.
        Eigen::Vector3d w_prev;

        bool stationary;

        /* FUNCTIONS */

        bool interpolate(const double t, Eigen::Vector3d& p, Eigen::Vector3d& v,
                         Eigen::Quaterniond& q, Eigen::Vector3d& w) {
            // Check if we're past the end of the trajectory.
            if (t > tf) {
                return false;
            }

            // If we are no longer between the current two points, we must
            // reinterpolate between the next two.
            if (!lerp.inrange(t)) {
                // Determine new points assuming constant waypoint step time.
                int idx = int((t - t0) / dt);
                mm_msgs::PoseTrajectoryPoint wp1 = waypoints[idx];
                mm_msgs::PoseTrajectoryPoint wp2 = waypoints[idx+1];

                double t1 = t0 + dt * idx;
                double t2 = t0 + dt * (idx + 1);

                Eigen::Vector3d p1, v1, w1;
                Eigen::Quaterniond q1;
                pose_traj_point_to_eigen(wp1, p1, q1, v1, w1);

                Eigen::Vector3d p2, v2, w2;
                Eigen::Quaterniond q2;
                pose_traj_point_to_eigen(wp2, p2, q2, v2, w2);

                lerp.interpolate(t1, t2, p1, p2, v1, v2);
                slerp.interpolate(t1, t2, q1, q2);

                w_prev = w1;
            }

            lerp.sample(t, p, v);
            slerp.sample(t, q);
            w = w_prev;
            return true;
        }

}; // class PoseTrajectory


} // namespace mm

#include "mm_motion_control/pose_control/trajectory.h"

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

#include <mm_msgs/PoseTrajectoryPoint.h>
#include <mm_msgs/PoseTrajectory.h>
#include <mm_msgs/conversions.h>
#include <mm_math_util/interp.h>


namespace mm {


PoseTrajectory::PoseTrajectory() {
    p_off = Eigen::Vector3d::Zero();
    stationary_position = Eigen::Vector3d::Zero();
    stationary_rotation = Eigen::Quaterniond(1, 0, 0, 0);
}

bool PoseTrajectory::follow(mm_msgs::PoseTrajectory trajectory) {
    t0 = ros::Time::now().toSec();
    dt = trajectory.dt.toSec();
    waypoints = trajectory.points;
    tf = t0 + (waypoints.size() - 1) * dt;

    w_prev = Eigen::Vector3d::Zero();
    stationary = false;

    return true;
}

void PoseTrajectory::offset(const Eigen::Vector3d& p) {
    p_off = p;
}

// If we're staying at a point, it is assumed the desired velocity is zero.
bool PoseTrajectory::stay_at(const Eigen::Vector3d& p,
                             const Eigen::Quaterniond& q) {
    // Clear existing trajectory.
    waypoints.clear();

    stationary = true;
    stationary_position = p;
    stationary_rotation = q;

    return true;
}

// Sample the trajectory at time t.
bool PoseTrajectory::sample(const double t, Eigen::Vector3d& p,
                            Eigen::Vector3d& v, Eigen::Quaterniond& q,
                            Eigen::Vector3d& w) {
    bool status = true;
    if (stationary) {
        p = stationary_position;
        v = Eigen::Vector3d::Zero();
        q = stationary_rotation;
        w = Eigen::Vector3d::Zero();
    } else {
        status = interpolate(t, p, v, q, w);
    }
    p += p_off;
    return status;
}

// Overloaded version in terms of poses and twists.
bool PoseTrajectory::sample(const double t, Eigen::Affine3d& pose,
                            Vector6d& twist) {
    Eigen::Vector3d p, v, w;
    Eigen::Quaterniond q;

    bool status = sample(t, p, v, q, w);
    pose = Eigen::Translation3d(p) * q;
    twist << v, w;

    return status;
}

bool PoseTrajectory::done(double t) {
    // Stationary trajectories don't end
    // Other trajectories end once their duration is expired
    return !stationary && t > tf;
}

bool PoseTrajectory::interpolate(const double t, Eigen::Vector3d& p,
                                 Eigen::Vector3d& v, Eigen::Quaterniond& q,
                                 Eigen::Vector3d& w) {
    // Check if we're past the end of the trajectory.
    if (done(t)) {
        // Return the last waypoint, but also signal that the
        // trajectory is done.
        mm_msgs::PoseTrajectoryPoint wp = waypoints.back();
        pose_traj_point_to_eigen(wp, p, q, v, w);
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

} // namespace mm

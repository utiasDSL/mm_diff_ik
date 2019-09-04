#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <mm_msgs/PoseTrajectoryPoint.h>
#include <mm_msgs/PoseTrajectory.h>
#include <geometry_msgs/Pose.h>


namespace mm {


void pose_traj_point_to_eigen(const mm_msgs::PoseTrajectoryPoint& msg,
                              Eigen::Vector3d& p, Eigen::Quaterniond& q,
                              Eigen::Vector3d& v, Eigen::Vector3d& w);

template <unsigned int N>
class CubicInterp {
    public:
        typedef Eigen::Matrix<double, N, 1> VectorNd;
        typedef Eigen::Matrix<double, 4, 1> Vector4d;
        typedef Eigen::Matrix<double, 4, 4> Matrix4d;
        typedef Eigen::Matrix<double, 4, N> Matrix4Nd;

        CubicInterp() {
            C = Matrix4Nd::Zero();
            t1 = 0;
            t2 = 0;
        }

        void interpolate(double t1, double t2, VectorNd& x1, VectorNd& x2,
                         VectorNd& dx1, VectorNd& dx2) {
            this->t1 = t1;
            this->t2 = t2;
            double dt = t2 - t1;

            this->x1 = x1;
            this->x2 = x2;
            this->dx1 = dx1;
            this->dx2 = dx2;

            Matrix4d A;
            A << 0, 0, 0, 1,
                 dt*dt*dt, dt*dt, dt, 1,
                 0,  0,  1,  0,
                 3*dt*dt,  2*dt,  1,  0;

            Matrix4Nd B;
            B << x1.transpose(),
                 x2.transpose(),
                 dx1.transpose(),
                 dx2.transpose();

            // Solve the system AC=B to get the coefficients of the cubic
            // polynomials.
            C = A.colPivHouseholderQr().solve(B);
        }

        // Sample the interpolated trajectory at time t.
        bool sample(const double t, VectorNd& x, VectorNd& dx) {
            double ta = t - t1;

            Vector4d T, dT;
            T  << ta*ta*ta, ta*ta, ta, 1;
            dT << 3*ta*ta, 2*ta, 1, 0;

            x = C.transpose() * T;
            dx = C.transpose() * dT;

            return inrange(t);
        }

        // Last point in the range.
        void last(VectorNd& x, VectorNd& dx) {
            x = x2;
            dx = dx2;
        }

        // True if t falls within the interpolation range, false otherwise
        bool inrange(const double t) {
            return t >= t1 && t <= t2;
        }

    private:
        // Coefficients of interpolated 3rd-order polynomials.
        Matrix4Nd C;

        // Start and end positions and velocities.
        VectorNd x1, x2, dx1, dx2;

        // Start and end times
        double t1, t2;
}; // class CubicInterp


// Linear spherical interpolation between quaternions.
class QuaternionInterp {
    public:
        QuaternionInterp() {}

        void interpolate(double t1, double t2, Eigen::Quaterniond& q1,
                         Eigen::Quaterniond& q2) {
            this->t1 = t1;
            this->t2 = t2;

            this->q1 = q1;
            this->q2 = q2;
        }

        bool sample(const double t, Eigen::Quaterniond& q) {
            double a = (t - t1) / (t2 - t1);
            q = q1.slerp(a, q2);
            return t >= t1 && t <= t2;
        }

    private:
        Eigen::Quaterniond q1, q2;

        double t1, t2;

}; // class QuaternionInterp


// TODO rename without interp---this doesn't do the raw interpolation processing!
// + actual interpolation code should be moved to the mm_math_util package
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

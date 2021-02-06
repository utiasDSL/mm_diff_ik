#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_msgs/Obstacle.h>


namespace mm {


const double OBS_SAFETY_DIST = 0.2;
const double OBS_INFLUENCE_DIST = 1.0;
const double OBS_COEFF = 1.0;


class ObstacleModel {
    public:
        // Initialize from centre point and radius.
        ObstacleModel(const Eigen::Vector2d p, const double r) {
            this->p = p;
            this->r = r;
        }

        // Initialize from an Obstacle message.
        ObstacleModel(const mm_msgs::Obstacle& msg) {
            p(0) = msg.centre.x;
            p(1) = msg.centre.y;
            r = msg.radius;
        }

        Eigen::Vector2d centre() const {
            return p;
        }

        double radius() const {
            return r;
        }

        // Returns true if the obstacle is within the influence distance of the
        // base, false otherwise.
        bool in_range(const Eigen::Vector2d& pb) const {
            Eigen::Vector2d n = p - pb;
            double d = n.norm() - BASE_RADIUS - r;
            return d < OBS_INFLUENCE_DIST;
        }

    private:
        Eigen::Vector2d p; // centre point
        double r; // radius

}; // class ObstacleModel model

} // namespace mm

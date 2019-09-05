#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_msgs/Obstacle.h>


namespace mm {

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

    private:
        Eigen::Vector2d p; // centre point
        double r; // radius

}; // class ObstacleModel model

} // namespace mm

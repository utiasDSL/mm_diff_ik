#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <geometry_msgs/WrenchStamped.h>


namespace mm {

class ForceBiasEstimator {
    public:
        ForceBiasEstimator() {}

        bool init(ros::NodeHandle& nh, size_t N) {
            this->N = N;
            count = 0;
            bias = Vector3d::Zero();
            sum = Vector3d::Zero();

            // TODO initialize stuff
        }

        void listen(const double dt, Eigen::Vector3d& bias) {
            ros::Rate rate(dt);
            while (ros::ok() && count < N) {
                ros::spinOnce();
                rate.sleep();
            }
            bias = sum / N;
        }

    private:
        Eigen::Vector3d bias;
        Eigen::Vector3d sum;

        void ft_cb(geometry_msgs::WrenchStamped& msg) {
            if (count < N) {
                Vector3d f;
                f << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z;
                sum += f;
                count += 1;
            } else {
                // TODO unsubscribe
            }
        }

}; // class FTBiasEstimator

} // namespace mm

#pragma once

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace rr {
    const uint32_t NUM_BASE_JOINTS = 3;
    const uint32_t NUM_ARM_JOINTS = 6;
    const uint32_t NUM_JOINTS = NUM_BASE_JOINTS + NUM_ARM_JOINTS;

    typedef Eigen::Matrix<double, NUM_JOINTS, 1> QVector;
    typedef Eigen::Matrix<double, 6, NUM_JOINTS> JacMatrix;
    typedef Eigen::Matrix<double, 6, 1> PoseVector;


    class ThingKinematics {
        public:
            // Calculate forward kinematics
            // TODO will certainly need to think about how to do this - what do
            // I actually want to be calculating?
            static void forward(QVector& q, PoseVector& P);

            // Calculate Jacobian.
            static void Jacobian(QVector& q, JacMatrix& J);

        private:
            // Calculate 4x4 transform from DH parameters.
            static void DH_tranform(double q, double d, double a, double alpha);
    }


    // class TransformLookup {
    //     public:
    //         TransformLookup();
    //
    //         // TODO need to confirm the direction of this transform
    //         void get(const std::string& target_frame,
    //                  const std::string& source_frame,
    //                  geometry_msgs::TransformStamped *T);
    //
    //     private:
    //         tf2_ros::Buffer *buffer;
    //         tf2_ros::TransformListener *tfl;
    // }

    class ForceControlDemo {
        public:
            ForceControlDemo();

            void loop();

        private:
            void optimize_kin();
            void parallel_control();
    };

}

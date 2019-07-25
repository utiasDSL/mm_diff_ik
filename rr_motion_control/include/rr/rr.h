#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>


namespace rr {
    const uint32_t NUM_BASE_JOINTS = 3;
    const uint32_t NUM_ARM_JOINTS = 6;
    const uint32_t NUM_JOINTS = NUM_BASE_JOINTS + NUM_ARM_JOINTS;

    typedef Eigen::Matrix<double, NUM_JOINTS, 1> QVector;
    typedef Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS> QMatrix;
    typedef Eigen::Matrix<double, 6, NUM_JOINTS> JacMatrix;
    //typedef Eigen::Matrix<double, 4, 4> Transform;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

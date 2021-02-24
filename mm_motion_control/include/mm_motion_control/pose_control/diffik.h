#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <mm_msgs/WrenchInfo.h>
#include <mm_msgs/Obstacles.h>

#include <mm_kinematics/kinematics.h>
#include <mm_motion_control/control.h>


namespace mm {

class DiffIKController : CartesianController {
    private:
        /** VARIABLES **/
        // Subscriber for information from the force/torque sensor.
        ros::Subscriber wrench_info_sub;

        // Subscriber for desired force.
        // TODO redundant: also contained in force info
        ros::Subscriber force_des_sub;

        // Subscriber for obstacle detections.
        ros::Subscriber obstacle_sub;

        double fd;
        Eigen::Vector3d force;
        Eigen::Vector3d torque;
        Eigen::Vector2d nf_xy;

        /** FUNCTIONS **/

        void wrench_info_cb(const mm_msgs::WrenchInfo& msg);
        void force_des_cb(const std_msgs::Float64 msg);
        void obstacle_cb(const mm_msgs::Obstacles& msg);

        int nullspace_manipulability(qpoases::QPData& qp1_data,
                                     const Eigen::VectorXd& u1,
                                     Eigen::VectorXd& u2);

        void calc_primary_objective(const ros::Time& now, JointMatrix& H,
                                    JointVector& g);

        void calc_joint_limits(JointVector& lb, JointVector& ub);

        int calc_obstacle_constraints(Eigen::MatrixXd& A, Eigen::VectorXd& b);
};

} // namespace mm

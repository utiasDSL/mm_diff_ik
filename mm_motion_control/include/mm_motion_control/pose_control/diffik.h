#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <mm_msgs/WrenchInfo.h>
#include <mm_msgs/Obstacles.h>

#include <mm_kinematics/kinematics.h>
#include <mm_optimization/qpoases.h>
#include <mm_control/cartesian/control.h>
#include <mm_motion_control/pose_control/obstacle.h>


namespace mm {

// Distances for the velocity damper constraints.
static const double M_PI_6 = M_PI / 6.0;
static const JointVector INFLUENCE_DIST((JointVector() <<
    0.5, 0.5, M_PI_6,                               /* base */
    M_PI_4, M_PI_4, M_PI_4, M_PI_4, M_PI_4, M_PI_4  /* arm */
).finished());

static const JointVector SAFETY_DIST = 0.25 * INFLUENCE_DIST;

// Make the joint true if a position limit should be enforced, false otherwise.
static const bool POSITIION_LIMITED[] = {
    false, false, false,               /* base */
    true, true, true, true, true, true /* arm  */
};

// Force values are in Newtons
static const double MAX_COMPLIANCE_FORCE = 100;
static const double FORCE_THRESHOLD = 5;

static const int NUM_WSR = 50; // max number of working set recalculations


class DiffIKController : public CartesianController {
    public:
        DiffIKController() {}
        ~DiffIKController() {}

        bool init(ros::NodeHandle& nh, const double hz);

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
        std::vector<ObstacleModel> obstacles;

        /** FUNCTIONS **/
        int update(const ros::Time& now);

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

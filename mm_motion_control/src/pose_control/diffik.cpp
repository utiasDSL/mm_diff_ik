#include "mm_motion_control/pose_control/control.h"

#include <Eigen/Eigen>
#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <mm_msgs/WrenchInfo.h>
#include <mm_msgs/Obstacles.h>

#include <mm_kinematics/kinematics.h>
#include <mm_control/control.h>
#include <mm_control/util/messages.h>


namespace mm {


bool DiffIKController::init(ros::NodeHandle& nh) {
    mm::CartesianController::init(nh);
    // TODO init subs/pubs and other variables
}


int DiffIKController::update(const ros::Time& now) {
    double t = now.toSec();
    Eigen::Affine3d Td;
    Vector6d Vd;
    trajectory.sample(t, Td, Vd);

    /*** OBJECTIVE ***/

    JointMatrix H;
    JointVector g;
    calc_objective(Td, Vd, q, dq, fd, force, torque, pc, dt, H, g);


    /*** BOUNDS ***/

    // Velocity damper inequality constraints
    JointVector u_lb, u_ub;
    calc_joint_limits(q, u_lb, u_ub);


    /*** CONSTRAINTS ***/

    // Obstacle constraints.
    Eigen::MatrixXd A_obs;
    Eigen::VectorXd ub_obs;
    Eigen::Vector2d pb(q[0], q[1]);
    calc_obstacle_limits(pb, obstacles, A_obs, ub_obs);


    /*** SOLVE QP ***/

    // TODO handle state
    qpoases::QProblem qp(NUM_JOINTS, A_obs.rows());
    qp.options.printLevel = qpOASES::PL_LOW;

    qp.data.H = H;
    qp.data.g = g;

    qp.data.lb = u_lb;
    qp.data.ub = u_ub;

    qp.data.A = A_obs;
    qp.data.ubA = ub_obs;
    qp.data.lbA.setConstant(A_obs.rows(), -qpOASES::INFTY);

    Eigen::VectorXd u1, u2;
    int status1 = qp.solve(u1);
    int status2 = nullspace_manipulability(q, qp.data, u1, dt, u2);
    u = u2;

    return status1;
}


// Control loop.
void DiffIKController::loop(const double hz) {
    ros::Rate rate(hz);
    ROS_INFO("Control loop started, waiting for trajectory...");

    while (ros::ok()) {
        ros::spinOnce();

        ros::Time now = ros::Time::now();
        double t = now.toSec();

        // When the trajectory ends, make sure to publish a zero velocity in
        // the case the trajectory itself doesn't end at zero.
        // TODO can I do this here or is the trajectory in a bad state?
        if (trajectory.done(t)) {
            ROS_INFO("Trajectory ended.");
            traj_active = false;
            continue;
        }

        // Do nothing if there is no trajectory active.
        if (!traj_active) {
            u = JointVector::Zero();
            publish_joint_speeds(now);
            rate.sleep();
            continue;
        }

        // Integrate using the model to get the best estimate of the state.
        // TODO: not sure if it would be better to use u instead of dq
        // -> would be less noisy
        // double dt = t - last_joint_state_time;
        // q = q + dt * dq;
        // last_joint_state_time = t;

        int status = update(now);

        // print but then publish 0
        // ROS_INFO_STREAM("u = " << u);
        // u = JointVector::Zero();

        publish_joint_speeds(now);
        publish_robot_state(now);

        rate.sleep();
    }
}


void DiffIKController::wrench_info_cb(const mm_msgs::WrenchInfo& msg) {
    force << msg.world.force.x, msg.world.force.y, msg.world.force.z;
    torque << msg.world.torque.x, msg.world.torque.y, msg.world.torque.z;
    // fd = msg.force_desired
}


void DiffIKController::force_des_cb(const std_msgs::Float64 msg) {
    fd = msg.data;
    ROS_INFO_STREAM("Set desired force = " << fd);
}


void DiffIKController::obstacle_cb(const mm_msgs::Obstacles& msg) {
    obstacles.clear();
    for (int i = 0; i < msg.obstacles.size(); ++i) {
        obstacles.push_back(ObstacleModel(msg.obstacles[i]));
    }
}

} // namespace mm

#include "mm_vicon/estimator/spoof.h"


using namespace Eigen;


namespace mm {


bool ViconEstimatorNodeSim::init(ros::NodeHandle& nh) {
    q = Eigen::Vector3d::Zero();
    dq = Eigen::Vector3d::Zero();

    rb_joint_vel_sub = nh.subscribe("/ridgeback_velocity_controller/cmd_vel", 1,
            &ViconEstimatorNodeSim::rb_joint_vel_cb, this);
    rb_joint_states_pub = nh.advertise<sensor_msgs::JointState>(
            "/rb_joint_states", 1);
}

void ViconEstimatorNodeSim::loop(const double hz) {
    ros::Rate rate(hz);

    while (ros::ok()) {
        ros::spinOnce();

        sensor_msgs::JointState rb_joint_states;
        rb_joint_states.header.stamp = ros::Time::now();

        rb_joint_states.position.push_back(q(0)); // x
        rb_joint_states.position.push_back(q(1)); // y
        rb_joint_states.position.push_back(q(2)); // yaw

        rb_joint_states.velocity.push_back(dq(0));
        rb_joint_states.velocity.push_back(dq(1));
        rb_joint_states.velocity.push_back(dq(2));

        rb_joint_states_pub.publish(rb_joint_states);

        rate.sleep();
    }
}

void ViconEstimatorNodeSim::rb_joint_vel_cb(const geometry_msgs::Twist& msg) {
    // Update time.
    double now = ros::Time::now().toSec();
    double dt = now - time_prev;
    time_prev = now;

    // Integrate using old velocity.
    q = q + dt * dq;

    // Update to new velocity.
    dq(0) = msg.linear.x;
    dq(1) = msg.linear.y;
    dq(2) = msg.angular.z;
}

} // namespace mm

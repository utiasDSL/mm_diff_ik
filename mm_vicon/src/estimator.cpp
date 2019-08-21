#include "mm_vicon/estimator.h"

#include <mm_math_util/wrap.h>
#include <mm_math_util/filter.h>


using namespace Eigen;


namespace mm {

bool ViconEstimatorNode::init(ros::NodeHandle& nh) {
    vicon_thing_base_sub = nh.subscribe(
            "/vicon/ThingBase/ThingBase", 1,
            &ViconEstimatorNode::vicon_thing_base_cb, this);

    rb_joint_states_pub = nh.advertise<sensor_msgs::JointState>(
            "/rb_joint_states", 1);

    new_msg = false;
    msg_count = 0;

    // Velocity is assumed to be 0 initially. Values for tau taken from
    // dsl__estimation__vicon package.
    filter_lin_vel.init(0.045, Vector2d::Zero());
    filter_rot_vel.init(0.025, 0);
}

void ViconEstimatorNode::loop(const double hz) {
    ros::Rate rate(hz);

    // Wait until the previous and current messages have been populated so that
    // calculations can be performed.
    while (ros::ok() && msg_count < 2) {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        ros::spinOnce();

        if (new_msg) {
            publish_joint_states();
            new_msg = false;
        }

        rate.sleep();
    }
}

void ViconEstimatorNode::vicon_thing_base_cb(const geometry_msgs::TransformStamped& msg) {
    tf_prev = tf_curr;
    tf_curr = msg;

    new_msg = true;
    ++msg_count;
}

void ViconEstimatorNode::publish_joint_states() {
    ros::Time t_prev = tf_prev.header.stamp;
    ros::Time t_curr = tf_curr.header.stamp;
    double dt = (t_curr - t_prev).toSec();

    double yaw_prev = tf::getYaw(tf_prev.transform.rotation);
    double yaw_curr = tf::getYaw(tf_curr.transform.rotation);

    Vector3d position_prev, position_curr;
    position_prev << tf_prev.transform.translation.x,
                     tf_prev.transform.translation.y,
                     tf_prev.transform.translation.z;
    position_curr << tf_curr.transform.translation.x,
                     tf_curr.transform.translation.y,
                     tf_curr.transform.translation.z;

    // Numerical differentiate to get velocity.
    Vector2d v_lin_meas;
    v_lin_meas << (position_curr(0) - position_prev(0)) / dt,
                  (position_curr(1) - position_prev(1)) / dt;

    // Note the fmod call to wrap to [-pi, pi].
    double vyaw_meas = wrap_to_pi(yaw_curr - yaw_prev) / dt;

    // Apply filtering on velocity.
    Vector2d v_lin = filter_lin_vel.next(v_lin_meas, dt);
    double vyaw = filter_rot_vel.next(vyaw_meas, dt);

    sensor_msgs::JointState rb_joint_states;
    rb_joint_states.header.stamp = ros::Time::now();

    // TODO publish names of states

    rb_joint_states.position.push_back(position_curr(0)); // x
    rb_joint_states.position.push_back(position_curr(1)); // y
    rb_joint_states.position.push_back(yaw_curr); // yaw

    rb_joint_states.velocity.push_back(v_lin(0));
    rb_joint_states.velocity.push_back(v_lin(1));
    rb_joint_states.velocity.push_back(vyaw);

    rb_joint_states_pub.publish(rb_joint_states);
}

} // namespace mm

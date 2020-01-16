#pragma once

// Basic proportional controller in joint space.

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <mm_kinematics/kinematics.h> // for the typedefs


namespace mm {

const static JointMatrix K = 0.5*JointMatrix::Identity();

const static JointVector HOME((JointVector()
            << 0.0, -1.0, M_PI_2,
                0.0, -0.75*M_PI, -M_PI_2, -0.75*M_PI, -M_PI_2, M_PI_2).finished());
const static double MAX_DQ = 0.2;


// Node for controlling the MM based on a desired joint trajectory.
class JointControlNode {
    public:
        JointControlNode() {}

        bool init(ros::NodeHandle& nh) {
            mm_joint_states_sub = nh.subscribe("/mm_joint_states", 1,
                    &JointControlNode::mm_joint_states_cb, this);

            ur10_joint_vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
                    "/ur_driver/joint_speed", 1);
            rb_joint_vel_pub = nh.advertise<geometry_msgs::Twist>(
                    "/ridgeback_velocity_controller/cmd_vel", 1);

            q_act = JointVector::Zero();
            q_des = HOME;

            joint_state_rec = false;
        }

        void loop(const double hz) {
            ros::Rate rate(hz);

            // Wait until we get at joint state message to know the actual
            // joint positions.
            while (ros::ok() && !joint_state_rec) {
                ros::spinOnce();
                rate.sleep();
            }

            // TODO hack to temporarily fix joint state initialization issue
            // should no longer be necessary---will test
            // ros::Duration(1.0).sleep();

            ROS_INFO("Joint control loop started, going home...");

            while (ros::ok()) {
                ros::spinOnce();

                JointVector q_err = q_des - q_act;

                // If we're close enough, stop.
                if (q_err.isZero(1e-3)) {
                    break;
                }

                // P control.
                JointVector dq_cmd = K * q_err;

                // Bound commands for base joints, which could be quite large.
                for (int i = 0; i < 3; ++i) {
                    if (dq_cmd(i) > MAX_DQ) {
                        dq_cmd(i) = MAX_DQ;
                    } else if (dq_cmd(i) < -MAX_DQ) {
                        dq_cmd(i) = -MAX_DQ;
                    }
                }

                publish_joint_speeds(dq_cmd);

                rate.sleep();
            }

            ROS_INFO("Done.");
        }

    private:
        /* VARIABLES */

        // Subscriber for current joint state of the robot.
        ros::Subscriber mm_joint_states_sub;

        // Subscriber for joint position commands.
        ros::Subscriber mm_joint_cmd_sub;

        // Publishers for joint speed commands.
        ros::Publisher ur10_joint_vel_pub;
        ros::Publisher rb_joint_vel_pub;

        JointVector q_act; // Actual joint positions.
        JointVector q_des; // Desired joint positions.

        bool joint_state_rec;

        /* FUNCTIONS */

        // TODO copied from control.h -- after ICRA I'll deal with this more
        // cleanly (or never touch it again)
        void publish_joint_speeds(const JointVector& dq_cmd) {
            // Split into base and arm joints to send out.
            Eigen::Vector3d dq_cmd_rb = dq_cmd.topRows<3>();
            Vector6d dq_cmd_ur10 = dq_cmd.bottomRows<6>();

            // Convert to JointTrajectory message with a single point (i.e.
            // velocity servoing) to publish to UR10.
            trajectory_msgs::JointTrajectoryPoint point;
            point.velocities = std::vector<double>(
                    dq_cmd_ur10.data(), dq_cmd_ur10.data() + dq_cmd_ur10.size());
            trajectory_msgs::JointTrajectory traj_ur10;
            traj_ur10.points.push_back(point);

            // Base twist.
            geometry_msgs::Twist twist_rb;
            twist_rb.linear.x = dq_cmd_rb(0);
            twist_rb.linear.y = dq_cmd_rb(1);
            twist_rb.angular.z = dq_cmd_rb(2);

            ur10_joint_vel_pub.publish(traj_ur10);
            rb_joint_vel_pub.publish(twist_rb);
        }

        void mm_joint_states_cb(const sensor_msgs::JointState& msg) {
            for (int i = 0; i < mm::NUM_JOINTS; ++i) {
                q_act(i) = msg.position[i];
            }
            joint_state_rec = true;
        }

        // void mm_joint_cmd_cb(const trajectory_msgs::JointTrajectoryPoint& msg) {
        //     for (int i = 0; i < mm::NUM_JOINTS; ++i) {
        //         q_des(i) = msg.positions[i];
        //     }
        // }
}; // class JointControlNode

}

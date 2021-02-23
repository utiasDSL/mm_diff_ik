#include <ros/ros.h>

#include <mm_kinematics/kinematics.h>

#include "mm_control/control.h"


static const double HZ = 125;

// Proportional gain matrix.
const static mm::JointMatrix K = 0.5 * mm::JointMatrix::Identity();

// Maximum joint speed.
const static double MAX_U = 0.2;

// Default home positions -- this can also be set by parameter.
static const mm::JointVector DEFAULT_HOME((mm::JointVector()
            << 0.0, 0.0, 0.0,
               0.0, -0.75*M_PI, -M_PI_2, -0.75*M_PI, -M_PI_2, M_PI_2).finished());


// Basic controller in joint space.
class JointController : mm::MMController {
    public:
        JointController() {}
        ~JointController() {}

        bool init(ros::NodeHandle& nh, mm::JointVector& qd) {
            // Initialize parent.
            mm::MMController::init(nh);

            this->qd = qd;
        }

        void loop(const double hz) {
            ros::Rate rate(hz);

            // Wait until we get at joint state message to know the actual
            // joint positions.
            while (ros::ok() && !joint_state_rec) {
                ros::spinOnce();
                rate.sleep();
            }

            while (ros::ok()) {
                ros::spinOnce();

                ros::Time now = ros::Time::now();

                // Error. If small enough, no need to do anything.
                mm::JointVector e = qd - q;
                if (e.isZero(1e-3)) {
                    break;
                }

                // Proportional control.
                u = K * e;

                // Bound commands. For base joints, these could be quite large.
                for (int i = 0; i < mm::NUM_JOINTS; ++i) {
                    if (u(i) > MAX_U) {
                        u(i) = MAX_U;
                    } else if (u(i) < -MAX_U) {
                        u(i) = -MAX_U;
                    }
                }

                publish_joint_speeds(now);

                rate.sleep();
            }
        }

    private:
        // Desired joint positions.
        mm::JointVector qd;

}; // class JointController


int main(int argc, char **argv) {
  ros::init(argc, argv, "home_node");
  ros::NodeHandle nh;

  mm::JointVector qd = DEFAULT_HOME;

  // If a home position was passed via parameter, use that as the desired
  // joint configuration.
  std::vector<double> joint_home_positions(mm::NUM_JOINTS);
  if (nh.getParam("/joint_home_positions", joint_home_positions)) {
      ROS_INFO_STREAM("Home joint configuration set from parameter.");
      qd = mm::JointVector(joint_home_positions.data());
  } else {
      ROS_INFO_STREAM("Using default home joint configuration.");
  }

  ROS_INFO("Going home...");

  JointController controller;
  controller.init(nh, qd);
  controller.loop(HZ);

  ROS_INFO("Done");

  return 0;
}

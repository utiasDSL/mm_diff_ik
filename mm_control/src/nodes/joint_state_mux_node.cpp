#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/JointState.h>

#include <mm_kinematics/kinematics.h>


static const double HZ = 125;


namespace mm {

// Combines /rb_joint_states and /ur10_joint_states messages into a single
// /mm_joint_states message.
class JointStateMux {
    public:
        JointStateMux() {}

        bool init(ros::NodeHandle& nh) {
            // TODO need to ensure this initializes properly
            position.reserve(9);
            velocity.reserve(9);

            ur10_joint_states_sub = nh.subscribe("/ur10_joint_states", 1,
                    &JointStateMux::ur10_joint_states_cb, this);
            rb_joint_states_sub = nh.subscribe("/rb_joint_states", 1,
                    &JointStateMux::rb_joint_states_cb, this);

            mm_joint_states_pub = nh.advertise<sensor_msgs::JointState>(
                    "/mm_joint_states", 1);

            rb_rec = false;
            ur10_rec = false;
        }

        // Update state of UR10 joints.
        void ur10_joint_states_cb(const sensor_msgs::JointState& msg) {
            for (int i = 0; i < msg.position.size(); ++i) {
                position[3+i] = msg.position[i];
                velocity[3+i] = msg.velocity[i];
            }
            ur10_rec = true;
        }

        // Update state of Ridgeback joints
        void rb_joint_states_cb(const sensor_msgs::JointState& msg) {
            for (int i = 0; i < msg.position.size(); ++i) {
                position[i] = msg.position[i];
                velocity[i] = msg.velocity[i];
            }
            rb_rec = true;
        }

        void publish_mm_joint_states() {
            sensor_msgs::JointState mm_joint_states;
            mm_joint_states.header.stamp = ros::Time::now();

            for (int i = 0; i < NUM_JOINTS; ++i) {
                // Bail in the case of bad position or velocity measurements
                // from the driver.
                if (std::abs(position[i]) > 2 * M_PI) {
                    ROS_WARN_STREAM(JOINT_NAMES[i] << " position = " << position[i]);
                    return;
                }
                if (std::abs(velocity[i]) > 5.0) {
                    ROS_WARN_STREAM(JOINT_NAMES[i] << " velocity = " << velocity[i]);
                    return;
                }
                mm_joint_states.name.push_back(JOINT_NAMES[i]);
                mm_joint_states.position.push_back(position[i]);
                mm_joint_states.velocity.push_back(velocity[i]);
            }

            mm_joint_states_pub.publish(mm_joint_states);
        }

        // Mux is ready if both Ridgeback and UR10 messages have been
        // received.
        bool ready() {
            return rb_rec && ur10_rec;
        }

    private:
        // Subscribe to current joint values of the robot.
        ros::Subscriber ur10_joint_states_sub;

        // Get (x, y, theta) from Vicon
        ros::Subscriber rb_joint_states_sub;

        ros::Publisher mm_joint_states_pub;

        std::vector<double> position;
        std::vector<double> velocity;

        bool rb_rec, ur10_rec;
}; // class JointStateMux

} // namespace mm


int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_state_mux_node");
  ros::NodeHandle nh;

  mm::JointStateMux mux;
  mux.init(nh);

  ros::Rate rate(HZ);

  // Wait until the mux is ready to begin publishing.
  while (ros::ok() && !mux.ready()) {
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok()) {
    ros::spinOnce();
    mux.publish_mm_joint_states();
    rate.sleep();
  }
}

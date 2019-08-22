#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <mm_kinematics/kinematics.h>


namespace mm {

class PosePublisher {
    public:
        PosePublisher() {}

        void init(ros::NodeHandle& nh) {
            mm_joint_states_sub = nh.subscribe("/mm_joint_states", 1,
                    &PosePublisher::mm_joint_states_cb, this);

            w_T_b_pub = nh.advertise<geometry_msgs::PoseStamped>(
                    "/pose/w_T_b", 1);
            w_T_e_pub = nh.advertise<geometry_msgs::PoseStamped>(
                    "/pose/w_T_e", 1);
        }

        void publish() {
            geometry_msgs::PoseStamped w_T_b_msg, w_T_e_msg;

            Eigen::Affine3d w_T_b;
            Eigen::Affine3d w_T_e;
            Kinematics::calc_w_T_b(q, w_T_b);
            Kinematics::calc_w_T_e(q, w_T_e);

            Eigen::Vector3d p_b = w_T_b.translation();
            Eigen::Quaterniond q_b(w_T_b.rotation());

            Eigen::Vector3d p_e = w_T_e.translation();
            Eigen::Quaterniond q_e(w_T_e.rotation());

            w_T_b_msg.pose.position.x = p_b(0);
            w_T_b_msg.pose.position.y = p_b(1);
            w_T_b_msg.pose.position.z = p_b(2);

            w_T_b_msg.pose.orientation.w = q_b.w();
            w_T_b_msg.pose.orientation.x = q_b.x();
            w_T_b_msg.pose.orientation.y = q_b.y();
            w_T_b_msg.pose.orientation.z = q_b.z();

            w_T_e_msg.pose.position.x = p_e(0);
            w_T_e_msg.pose.position.y = p_e(1);
            w_T_e_msg.pose.position.z = p_e(2);

            w_T_e_msg.pose.orientation.w = q_e.w();
            w_T_e_msg.pose.orientation.x = q_e.x();
            w_T_e_msg.pose.orientation.y = q_e.y();
            w_T_e_msg.pose.orientation.z = q_e.z();

            ros::Time now = ros::Time::now();
            w_T_b_msg.header.stamp = now;
            w_T_e_msg.header.stamp = now;

            w_T_b_pub.publish(w_T_b_msg);
            w_T_e_pub.publish(w_T_e_msg);
        }

    private:
        ros::Subscriber mm_joint_states_sub;

        ros::Publisher w_T_b_pub;
        ros::Publisher w_T_e_pub;

        JointVector q;

        void mm_joint_states_cb(const sensor_msgs::JointState& msg) {
            for (int i = 0; i < msg.position.size(); ++i) {
                q(i) = msg.position[i];
            }
        }

}; // class PosePublisher

} // namespace mm


int main(int argc, char **argv) {
  ros::init(argc, argv, "mm_pose_publisher_node");
  ros::NodeHandle nh;

  mm::PosePublisher pose_pub;
  pose_pub.init(nh);

  ros::Rate rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    pose_pub.publish();
    rate.sleep();
  }
}

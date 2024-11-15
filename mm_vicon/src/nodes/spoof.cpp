#include "mm_vicon/estimator/spoof.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "mm_vicon_estimator_node");
  ros::NodeHandle nh;

  mm::ViconEstimatorNodeSim node;
  node.init(nh);
  node.loop(100); // 100 Hz - nominal Vicon frequency.

  return 0;
}

#!/bin/sh

rosbag record -o mm   \
  /mm_pose_state      \
  /optimization_state \
  /joint_states       \
  /vicon/ThingBase/ThingBase

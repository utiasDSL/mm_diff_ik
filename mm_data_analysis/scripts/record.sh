#!/bin/sh

rosbag record -o $1      \
  /mm_pose_state         \
  /optimization_state    \
  /mm_joint_states       \
  /ur_driver/joint_speed \
  /ridgeback_velocity_controller/cmd_vel \
  /vicon/ThingBase/ThingBase

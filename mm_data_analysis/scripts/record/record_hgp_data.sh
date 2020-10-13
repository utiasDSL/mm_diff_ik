#!/bin/sh

rosbag record -o "$1"    \
  /mm_joint_states       \
  /ur_driver/joint_speed \
  /ridgeback_velocity_controller/cmd_vel \
  /ridgeback_velocity_controller/odom \
  /joy/cmd_vel \
  /vicon/ThingBase2/ThingBase2 \
  /vicon/ThingEE/ThingEE \
  /vicon/markers

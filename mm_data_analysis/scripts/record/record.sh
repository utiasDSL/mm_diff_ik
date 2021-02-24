#!/bin/sh

rosbag record -o "$1"    \
  /mm_pose_state         \
  /mm_control_state         \
  /mm_joint_states       \
  /ur_driver/joint_speed \
  /ur10_joint_states \
  /ridgeback_velocity_controller/cmd_vel \
  /vicon/ThingBase2/ThingBase2 \
  /vicon/Board/Board \
  /vicon/Barrel/Barrel \
  /vicon/TrayTest/TrayTest \
  /vicon/markers \
  /robotiq_force_torque_wrench \
  /force/info \
  /mm_wrench/info \
  /force/desired \
  /front/scan \
  /obstacles

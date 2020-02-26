#!/bin/sh

rosbag record -o "$1"    \
  /mm_pose_state         \
  /optimization_state    \
  /mm_joint_states       \
  /ur_driver/joint_speed \
  /ridgeback_velocity_controller/cmd_vel \
  /vicon/ThingBase/ThingBase \
  /vicon/Board/Board \
  /vicon/Barrel/Barrel \
  /vicon/markers \
  /robotiq_force_torque_wrench \
  /force_control/state \
  /force/info \
  /force/desired \
  /obstacles

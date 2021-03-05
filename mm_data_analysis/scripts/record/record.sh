#!/bin/sh

rosbag record -o "$1"    \
  /mm/joint_states       \
  /mm/control/cartesian/info \
  /mm/control/cartesian/trajectory \
  /mm/control/cartesian/point \
  /mm/control/joint/info \
  /mm/control/joint/trajectory \
  /mm/control/joint/point \
  /mm/obstacles \
  /mm/force_torque/info \
  /mm/force_torque/desired \
  /ur_driver/joint_speed \
  /ur10_joint_states \
  /ridgeback_velocity_controller/cmd_vel \
  /vicon/ThingBase2/ThingBase2 \
  /vicon/Board/Board \
  /vicon/Barrel/Barrel \
  /vicon/TrayTest/TrayTest \
  /vicon/markers \
  /robotiq_force_torque_wrench \
  /front/scan

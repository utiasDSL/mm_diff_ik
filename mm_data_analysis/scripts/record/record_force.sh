#!/bin/sh

rosbag record -o "$1"    \
  /robotiq_force_torque_wrench \
  /force/info \
  /wrench

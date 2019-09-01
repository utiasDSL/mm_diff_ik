#!/bin/bash

# TODO use a regex for this

# Log all of the data in a rosbag.
rosbag record /log/time \
              /log/force_raw \
              /log/force \
              /log/pos_offset \
              /log/ee_world_des \
              /log/ee_world_act \
              /log/ee_arm_des \
              /log/ee_arm_act \
              /log/base_des \
              /log/base_act \
              /vel_based_pos_traj_controller/state \
              -o $1

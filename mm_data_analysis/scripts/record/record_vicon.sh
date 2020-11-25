#!/bin/sh

rosbag record -o "$1" --regex "/vicon/(.*)"

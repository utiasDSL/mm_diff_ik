#!/bin/sh

# Play back the force info messages from a previous rosbag.
rosbag play --topic /force/info $1

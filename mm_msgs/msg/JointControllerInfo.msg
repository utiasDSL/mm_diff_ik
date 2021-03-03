# This is very similar to control_msgs/JointTrajectoryControllerState, except
# we also include the joint commands and don't bother naming the joints.

Header header

trajectory_msgs/JointTrajectoryPoint actual
trajectory_msgs/JointTrajectoryPoint desired
trajectory_msgs/JointTrajectoryPoint error

# Commanded joint velocities
float64[] command

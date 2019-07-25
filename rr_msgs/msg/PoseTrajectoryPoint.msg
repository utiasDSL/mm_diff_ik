# Pose and velocity waypoint for trajectory in task space. We don't include
# acceleration since control is done on the velocity level.
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
duration time_from_start

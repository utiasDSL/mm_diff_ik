Header header

# End effector pose
mm_msgs/CartesianTrajectoryPoint actual
mm_msgs/CartesianTrajectoryPoint desired
mm_msgs/CartesianTrajectoryPoint error

# Joint position, actual velocity, and commanded velocity
sensor_msgs/JointState joints
float64[] command

# Optimization info: TODO this would be better as its own message
# float64[] x
# float64 objective
# int64 ret
# int64 status

Header header

# End effector pose
geometry_msgs/Pose actual
geometry_msgs/Pose desired
geometry_msgs/Pose error

# End effector twist
geometry_msgs/Twist twist_actual
geometry_msgs/Twist twist_desired
geometry_msgs/Twist twist_error

# Joint position, actual velocity, and commanded velocity
float64[] q
float64[] dq
float64[] u

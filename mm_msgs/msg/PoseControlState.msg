Header header

# End effector pose
geometry_msgs/Pose actual
geometry_msgs/Pose desired
geometry_msgs/Pose error

# End effector twist
geometry_msgs/Twist twist_actual
geometry_msgs/Twist twist_desired
geometry_msgs/Twist twist_error

# Rotation error used for control is not necessarily the same as the actual
# orientation error. This is the former.
geometry_msgs/Vector3 rotation_error

# Joint position, actual velocity, and commanded velocity
float64[] q
float64[] dq
float64[] u

# Optimization info: TODO this would be better as its own message
float64[] x
float64 objective
int64 ret
int64 status

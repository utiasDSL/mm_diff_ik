import rospy
from geometry_msgs.msg import Vector3, Quaternion, Pose
from mm_msgs.msg import CartesianTrajectoryPoint


def quat_msg(q):
    ''' Generate Quaternion message from array of [x, y, z, w]. '''
    msg = Quaternion()
    msg.x = q[0]
    msg.y = q[1]
    msg.z = q[2]
    msg.w = q[3]
    return msg


def vec3_msg(v):
    ''' Generate Vector3 message from array of [x, y, z]. '''
    msg = Vector3()
    msg.x = v[0]
    msg.y = v[1]
    msg.z = v[2]
    return msg


def pose_msg(p, q):
    ''' Generate a Pose message from position and orientation. '''
    msg = Pose()
    msg.position = vec3_msg(p)
    msg.orientation = quat_msg(q)
    return msg


def waypoint_msg(t, p, v, a, q, w, alpha):
    ''' Generate PoseTrajectoryPoint message from time, position, orientation,
        and linear and angular velocities. '''
    waypoint = CartesianTrajectoryPoint()
    waypoint.time = rospy.Duration(t)

    waypoint.state.pose = pose_msg(p, q)

    waypoint.state.twist.linear = vec3_msg(v)
    waypoint.state.twist.angular = vec3_msg(w)

    waypoint.state.acceleration.linear = vec3_msg(a)
    waypoint.state.acceleration.angular = vec3_msg(alpha)

    return waypoint

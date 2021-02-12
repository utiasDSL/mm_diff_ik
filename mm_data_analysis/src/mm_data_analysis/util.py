import numpy as np
import tf.transformations as tfs
import glob
import os
import sys


def rms(e):
    ''' Calculate root mean square of a vector of data. '''
    return np.sqrt(np.mean(np.square(e)))


def wrap_to_pi(angle):
    """Wrap the angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def msg_time(msg):
    """Extract message timestamp as float in seconds."""
    return msg.header.stamp.to_sec()


def parse_t0(msgs):
    return msg_time(msgs[0])


def parse_time(msgs, normalize_time=True, t0=None):
    """Parse time in seconds from a list of messages.

    If normalize_time is True (the default), the array of time values will be
    normalized such that t[0] = t0. If t0 is not provided, it defaults to t[0].
    """
    t = np.array([msg_time(msg) for msg in msgs])
    if normalize_time:
        if t0:
            t -= t0
        else:
            t -= t[0]
    return t


def vec3_list_to_np(vecs):
    """Convert list of Vector3 messages to an Nx3 numpy array."""
    return np.array([[v.x, v.y, v.z] for v in vecs])


def vec3_to_np(msg):
    """Convert Vector3 message to numpy array."""
    return np.array([msg.x, msg.y, msg.z])


def quat_to_np(msg):
    """Convert Quaternion message to numpy array."""
    return np.array([msg.x, msg.y, msg.z, msg.w])


def quat_error(q1, q2):
    """Return the quaternion representing difference between q1 and q2.

    This is equivalent to q1 * q2^{-1}, where * is quaternion multiplication.
    """
    return tfs.quaternion_multiply(q1, tfs.quaternion_inverse(q2))


def quat_angle(q):
    """Return the scalar angle of the axis-angle represented by this quaternion."""
    v = q[:3]  # vector part
    w = q[3]   # scalar part
    angle = 2 * np.arctan2(np.linalg.norm(v), w)
    return wrap_to_pi(angle)


def trim_to_traj(msgs, pose_msgs):
    """Trim the list of msgs such that it only contains those messages which
       fall within the trajectory."""
    t0 = msg_time(pose_msgs[0])
    tf = msg_time(pose_msgs[-1])
    return [msg for msg in msgs if msg_time(msg) >= t0 and msg_time(msg) <= tf]


# TODO refactor this
def align_lists(t1, l1, t2, l2):
    i1 = i2 = 0
    aligned = []
    for i1 in xrange(len(l1)):
        t = t1[i1]
        while i2 + 1 < len(l2) and t2[i2 + 1] < t:
            i2 += 1
        # aligned.append((l1[i1], l2[i2]))
        aligned.append(l2[i2])
    return aligned


def _slerp_quaternions(q1, q2, s):
    """Spherical linear interpolation for quaternions."""
    return tfs.quaternion_slerp(q1, q2, s)


def _lerp_vectors(v1, v2, s):
    """Linear interpolation function."""
    return (1 - s) * v1 + s * v2


def linear_interpolate_list(interp_times, times, values, interp_func=_lerp_vectors):
    """Interpolate values at given interpolation times."""
    item_idx = 0
    interpolated_values = []
    for ti in interp_times:
        if ti < times[0]:
            # if interpolation times fall before the time of the first value,
            # we just use that one without any interpolation
            interpolated_values.append(values[0])
            continue
        elif ti > times[-1]:
            # likewise for interpolation times after the last value
            interpolated_values.append(values[-1])
            continue

        # find the next item that comes at or after time ti
        while times[item_idx] < ti:
            item_idx += 1

        if item_idx > 0:
            # times before and after the desired interpolation time ti
            ta = times[item_idx - 1]
            tb = times[item_idx]
            s = (ti - ta) / (tb - ta)

            a = values[item_idx - 1]
            b = values[item_idx]

            item_interp = interp_func(a, b, s)

            interpolated_values.append(item_interp)

    return interpolated_values


def spherical_interpolate_quaternions(interp_times, times, quats):
    return linear_interpolate_list(interp_times, times, quats, interp_func=_slerp_quaternions)


# TODO this is broken: commenting out until fixed so not used
# def align_msgs(msgs1, msgs2):
#     ''' Align messages based on time stamp. Messages must have a header.stamp
#         field. msgs2 are aligned with msgs1. '''
#     t0 = parse_t0(msgs1)
#     t1 = parse_time(msgs1)
#     t2 = parse_time(msgs2, t0=t0)
#     return align_lists(t1, msgs1, t2, msgs2)


def most_recent_file(pattern):
    """Return most recently modified file matching pattern.

    e.g. most_recent_file('*.bag') will return the most recently modified
         bag file in the directory.
    """
    return max(glob.iglob(pattern), key=os.path.getmtime)


def arg_or_most_recent(pattern):
    """Return the first argument, if provided, or the most recent file matching
       pattern otherwise.
    """
    if len(sys.argv) > 1:
        fname = sys.argv[1]
    else:
        fname = most_recent_file('*.bag')
    return fname

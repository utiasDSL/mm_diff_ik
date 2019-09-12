import numpy as np


def rms(e):
    ''' Calculate root mean square of a vector of data. '''
    return np.sqrt(np.mean(np.square(e)))


def parse_time(msgs):
    ''' Parse time in seconds from a list of messages and normalize so t[0] = 0. '''
    t = np.array([msg.header.stamp.to_sec() for msg in msgs])
    t -= t[0]
    return t


def vec3_msg_to_np(vecs):
    ''' Convert a list of Vector3 messages to a Nx3 numpy array. '''
    return np.array([[v.x, v.y, v.z] for v in vecs])


def trim_to_traj(msgs, pose_msgs):
    ''' Trim the list of msgs such that it only contains those messages which
        fall within the trajectory. '''
    t0 = pose_msgs[0].header.stamp.to_sec()
    tf = pose_msgs[-1].header.stamp.to_sec()

    # find index of first message in the trajectory
    for i in xrange(len(msgs)):
        if msgs[i].header.stamp.to_sec() > t0:
            i_0 = i
            break

    # find index of last message in the trajectory
    for i in xrange(len(msgs)):
        if msgs[i].header.stamp.to_sec() > tf:
            i_f = i - 1
            break

    return msgs[i_0:i_f+1]

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

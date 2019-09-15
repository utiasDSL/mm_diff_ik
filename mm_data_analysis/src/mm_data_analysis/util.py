import numpy as np


def rms(e):
    ''' Calculate root mean square of a vector of data. '''
    return np.sqrt(np.mean(np.square(e)))


def parse_time(msgs, normalize_time=True):
    ''' Parse time in seconds from a list of messages and normalize so t[0] = 0. '''
    t = np.array([msg.header.stamp.to_sec() for msg in msgs])
    if normalize_time:
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


def align_lists(t1, l1, t2, l2):
    i1 = i2 = 0
    aligned = []
    for i1 in xrange(len(l1)):
        t = t1[i1]
        while i2 + 1 < len(l2) and t2[i2 + 1] < t:
            i2 += 1
        aligned.append((l1[i1], l2[i2]))
    return aligned


def align_msgs(msgs1, msgs2):
    ''' Align messages based on time stamp. Messages must have a header.stamp
        field. msgs2 are aligned with msgs1. '''
    t1 = parse_time(msgs1)
    t2 = parse_time(msgs2)
    return align_lists(t1, msgs1, t2, msgs2)


# def align_msgs(msgs1, msgs2):
#     ''' Align messages based on time stamp. Messages must have a header.stamp
#         field. msgs2 are aligned with msgs1. '''
#     i1 = i2 = 0
#     aligned_msgs = []
#     for i1 in xrange(len(msgs1)):
#         t = msgs1[i1].header.stamp.to_sec()
#         while i2 + 1 < len(msgs2) and msgs2[i2 + 1].header.stamp.to_sec() < t:
#             i2 += 1
#         aligned_msgs.append((msgs1[i1], msgs2[i2]))
#     return aligned_msgs

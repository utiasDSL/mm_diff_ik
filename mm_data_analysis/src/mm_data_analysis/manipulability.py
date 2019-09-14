import numpy as np
import rosbag
import mm_data_analysis.util as util
import mm_kinematics.kinematics as kinematics


def calc_mi(joint_msgs, pose_msgs):
    joint_msgs = util.trim_to_traj(joint_msgs, pose_msgs)
    t = util.parse_time(joint_msgs)
    qs = np.array([msg.position for msg in joint_msgs])

    # calculate MI at each timestep
    mi = np.zeros(len(t))
    for i in xrange(len(mi)):
        mi[i] = kinematics.manipulability(qs[i,:])
    return t, mi


def parse_bag(name):
    bag = rosbag.Bag(name)
    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]

    t0 = pose_msgs[0].header.stamp.to_sec()
    tf = pose_msgs[-1].header.stamp.to_sec()
    freq = len(pose_msgs) / (tf - t0)

    t, mi = calc_mi(joint_msgs, pose_msgs)
    return t, mi, freq

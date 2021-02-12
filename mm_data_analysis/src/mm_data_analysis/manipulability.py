import rosbag
import mm_data_analysis.util as util


def parse_bag(model, bagname):
    bag = rosbag.Bag(bagname)

    pose_msgs = [msg for _, msg, _ in bag.read_messages('/mm_pose_state')]
    joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]
    joint_msgs_traj = util.trim_to_traj(joint_msgs, pose_msgs)

    t0 = util.msg_time(pose_msgs[0])
    t = util.parse_time(joint_msgs_traj, t0=t0)

    ms = [model.manipulability(msg.position) for msg in joint_msgs_traj]

    return t, ms

#!/usr/bin/env python2
import numpy as np
import rosbag
import mm_kinematics.kinematics as kinematics
import mm_data_analysis.util as util
import tf.transformations as tfs

import IPython


BAG = 'bags/2019-09-15/slalom/line/line_v0.4_slalom_2019-09-15-14-10-43.bag'


def main():
    bag = rosbag.Bag(BAG)
    joint_msgs = [msg for _, msg, _ in bag.read_messages('/mm_joint_states')]
    qs = [msg.position for msg in joint_msgs]

    ts = util.parse_time(joint_msgs)
    # dt = ts[1:] - ts[:-1]

    pbs = []  # base positions
    for q in qs:
        Ts = kinematics.forward_chain(q)
        w_T_b = Ts[3]
        w_p_b = tfs.translation_from_matrix(w_T_b)
        pbs.append(w_p_b)

    pbs = np.array(pbs)

    v = np.zeros(len(joint_msgs)-1)
    for i in xrange(1, len(joint_msgs)):
        dt = ts[i] - ts[i-1]
        dp = pbs[i,:] - pbs[i-1,:]
        v[i-1] = np.linalg.norm(dp) / dt

    print(np.max(v))


if __name__ == '__main__':
    main()

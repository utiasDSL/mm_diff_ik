#!/usr/bin/env python2
from __future__ import print_function

import sys

import rosbag
import matplotlib.pyplot as plt
import mm_data_analysis.util as util


def main():
    bag = rosbag.Bag(sys.argv[1])
    control_msgs = [
        msg for _, msg, _ in bag.read_messages("/mm/control/cartesian/info")
    ]

    t = util.parse_time(control_msgs)

    # desired pose
    pds = util.vec3_list_to_np([msg.desired.pose.position for msg in control_msgs])
    qds = util.vec3_list_to_np([msg.desired.pose.orientation for msg in control_msgs])

    # actual pose
    pas = util.vec3_list_to_np([msg.actual.pose.position for msg in control_msgs])
    qas = util.vec3_list_to_np([msg.actual.pose.orientation for msg in control_msgs])

    # error pose
    pes = util.vec3_list_to_np([msg.error.pose.position for msg in control_msgs])
    qes = util.vec3_list_to_np([msg.error.pose.orientation for msg in control_msgs])

    # desired linear velocity
    vds = util.vec3_list_to_np([msg.desired.twist.linear for msg in control_msgs])
    wds = util.vec3_list_to_np([msg.desired.twist.angular for msg in control_msgs])

    # actual twist
    # TODO: this is not actually published by the controller at the moment
    vas = util.vec3_list_to_np([msg.actual.twist.linear for msg in control_msgs])
    was = util.vec3_list_to_np([msg.actual.twist.angular for msg in control_msgs])

    plt.figure()
    plt.plot(t, pds[:, 0], "--", color="C0", label="$p_{x,d}$")
    plt.plot(t, pds[:, 1], "--", color="C1", label="$p_{y,d}$")
    plt.plot(t, pds[:, 2], "--", color="C2", label="$p_{z,d}$")
    plt.plot(t, pas[:, 0], color="C0", label="$p_{x}$")
    plt.plot(t, pas[:, 1], color="C1", label="$p_{y}$")
    plt.plot(t, pas[:, 2], color="C2", label="$p_{z}$")
    plt.grid()
    plt.legend()
    plt.title("End Effector Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")

    plt.figure()
    plt.plot(t, qds[:, 0], "--", color="C0", label="$q_{x,d}$")
    plt.plot(t, qds[:, 1], "--", color="C1", label="$q_{y,d}$")
    plt.plot(t, qds[:, 2], "--", color="C2", label="$q_{z,d}$")
    plt.plot(t, qas[:, 0], color="C0", label="$q_{x}$")
    plt.plot(t, qas[:, 1], color="C1", label="$q_{y}$")
    plt.plot(t, qas[:, 2], color="C2", label="$q_{z}$")
    plt.grid()
    plt.legend()
    plt.title("End Effector Orientation")
    plt.xlabel("Time (s)")
    plt.ylabel("Quaternion unit")

    plt.figure()
    plt.plot(t, vds[:, 0], "--", color="C0", label="$v_{x,d}$")
    plt.plot(t, vds[:, 1], "--", color="C1", label="$v_{y,d}$")
    plt.plot(t, vds[:, 2], "--", color="C2", label="$v_{z,d}$")
    # plt.plot(t, vas[:, 0], color="C0", label="$v_{x}$")
    # plt.plot(t, vas[:, 1], color="C1", label="$v_{y}$")
    # plt.plot(t, vas[:, 2], color="C2", label="$v_{z}$")
    plt.grid()
    plt.legend()
    plt.title("End Effector Linear Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")

    plt.figure()
    plt.plot(t, wds[:, 0], "--", color="C0", label="$w_{x,d}$")
    plt.plot(t, wds[:, 1], "--", color="C1", label="$w_{y,d}$")
    plt.plot(t, wds[:, 2], "--", color="C2", label="$w_{z,d}$")
    # plt.plot(t, was[:, 0], color="C0", label="$w_x$")
    # plt.plot(t, was[:, 1], color="C1", label="$w_y$")
    # plt.plot(t, was[:, 2], color="C2", label="$w_z$")
    plt.grid()
    plt.legend()
    plt.title("End Effector Angular Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular velocity (rad/s)")

    plt.figure()
    plt.plot(t, pes[:, 0], color="C0", label="$p_{x}$")
    plt.plot(t, pes[:, 1], color="C1", label="$p_{y}$")
    plt.plot(t, pes[:, 2], color="C2", label="$p_{z}$")
    plt.grid()
    plt.legend()
    plt.title("End Effector Position Error")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")

    plt.figure()
    plt.plot(t, qes[:, 0], color="C0", label="$q_{x}$")
    plt.plot(t, qes[:, 1], color="C1", label="$q_{y}$")
    plt.plot(t, qes[:, 2], color="C2", label="$q_{z}$")
    plt.grid()
    plt.legend()
    plt.title("End Effector Orientation Error")
    plt.xlabel("Time (s)")
    plt.ylabel("Quaternion unit")

    plt.show()


if __name__ == "__main__":
    main()

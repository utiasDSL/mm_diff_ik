#!/usr/bin/env python2
from __future__ import print_function


import rospy
import matplotlib.pyplot as plt
import numpy as np


def stamp_to_float(stamp):
    ''' Convert a ros timestamp message to a float in seconds. '''
    return rospy.Time(secs=stamp.secs, nsecs=stamp.nsecs).to_time()


def parse_time(msgs):
    t = np.array([stamp_to_float(msg.header.stamp) for msg in msgs])
    t -= t[0]
    return t


def plot_pose_error(pose_msgs):
    t = parse_time(pose_msgs)
    pos_errs = [msg.error.position for msg in pose_msgs]
    quat_errs = [msg.error.orientation for msg in pose_msgs]

    px = [p.x for p in pos_errs]
    py = [p.y for p in pos_errs]
    pz = [p.z for p in pos_errs]

    qx = [q.x for q in quat_errs]
    qy = [q.y for q in quat_errs]
    qz = [q.z for q in quat_errs]

    plt.figure()
    plt.plot(t, px, label='x')
    plt.plot(t, py, label='y')
    plt.plot(t, pz, label='z')
    plt.grid()
    plt.legend()
    plt.title('Linear End Effector Pose Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')

    plt.figure()
    plt.plot(t, qx, label='x')
    plt.plot(t, qy, label='y')
    plt.plot(t, qz, label='z')
    plt.grid()
    plt.legend()
    plt.title('Rotational End Effector Pose Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (rad)')


def plot_manipulability(mm_joint_states_msgs):
    t = parse_time(mm_joint_states_msgs)
    qs = np.array([msg.position for msg in mm_joint_states_msgs])

    plt.figure()
    plt.plot()
    plt.grid()
    plt.legend()
    plt.title('Manipulability Index')
    plt.xlabel('Time (s)')
    plt.ylabel('Manipulability Index')


def plot_joints(mm_joint_states_msgs, idx=range(9)):
    t = parse_time(mm_joint_states_msgs)
    qs = np.array([msg.position for msg in mm_joint_states_msgs])
    # TODO need to figure out the masking

    plt.figure()
    plt.plot()
    plt.grid()
    plt.legend()
    plt.title('Joint Positions')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')

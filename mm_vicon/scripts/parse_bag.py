import sys
import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tfs

import IPython


def stamp_to_float(stamp):
    ''' Convert a ros timestamp message to a float in seconds. '''
    return rospy.Time(secs=stamp.secs, nsecs=stamp.nsecs).to_time()


def parse_vic_msgs(vic_msgs):
    t = np.array([stamp_to_float(msg.header.stamp) for msg in vic_msgs])
    t -= t[0]

    x = [msg.transform.translation.x for msg in vic_msgs]
    y = [msg.transform.translation.y for msg in vic_msgs]

    # tf quaternion is [x, y, z, w]
    quats = [[msg.transform.rotation.x, msg.transform.rotation.y,
              msg.transform.rotation.z, msg.transform.rotation.w] for msg in vic_msgs]
    rpys = [tfs.euler_from_quaternion(q) for q in quats]
    yaw = [rpy[2] for rpy in rpys]

    vx = np.zeros(len(t))
    vy = np.zeros(len(t))

    # experiments with exponential smoothing
    vyaw = np.zeros(len(yaw))
    vyaw_meas = np.zeros(len(yaw))
    vyaw[0] = 0  # start at zero velocity
    tau = 0.025

    for i in xrange(1, len(t)):
        dt = t[i] - t[i-1]

        vx[i] = (x[i] - x[i-1]) / dt
        vy[i] = (y[i] - y[i-1]) / dt

        c = 1 - np.exp(-dt/tau)
        vyaw_meas[i] = (yaw[i] - yaw[i-1]) / dt
        vyaw[i] = c * vyaw_meas[i] + (1-c) * vyaw[i-1]


    # plt.plot(t, yaw, label='yaw')
    # plt.plot(t, vyaw_meas, label='vyaw raw')
    # plt.plot(t, vyaw, label='vyaw')

    # alternatively, we can calculate the difference between quaternions
    vyaw2 = np.zeros(len(yaw))
    for i in xrange(1, len(t)):
        dt = t[i] - t[i-1]
        dq = tfs.quaternion_multiply(tfs.quaternion_inverse(quats[i-1]), quats[i])
        drpy = tfs.euler_from_quaternion(dq)
        vyaw2[i] = drpy[2] / dt

    return t, vx, vy, vyaw_meas

    # plt.plot(t, vyaw2, label='vyaw 2')
    #
    # plt.legend()
    # plt.show()


def parse_estimator_msgs(rb_state_msgs):
    t = np.array([stamp_to_float(msg.header.stamp) for msg in rb_state_msgs])
    t -= t[0]

    x = [msg.position[0] for msg in rb_state_msgs]
    y = [msg.position[1] for msg in rb_state_msgs]
    yaw = [msg.position[2] for msg in rb_state_msgs]

    vx = [msg.velocity[0] for msg in rb_state_msgs]
    vy = [msg.velocity[1] for msg in rb_state_msgs]
    vyaw = [msg.velocity[2] for msg in rb_state_msgs]

    return t, vx, vy, vyaw

    # TODO plot the raw direct from Vicon, calculated in this file

    # plt.figure()
    # plt.subplot(2, 1, 1)
    # plt.plot(x, y)
    # plt.xlabel('x (m)')
    # plt.ylabel('y (m)')
    # plt.grid()
    #
    # plt.subplot(2, 1, 2)
    # plt.plot(t, yaw)
    # plt.xlabel('t (sec)')
    # plt.ylabel('yaw (rad)')

    # plt.figure()
    # plt.subplot(3, 1, 1)
    # plt.plot(t, vx)
    # plt.subplot(3, 1, 2)
    # plt.plot(t, vy)
    # plt.subplot(3, 1, 3)
    # plt.plot(t, vyaw)
    #
    # plt.show()


def comp_raw_filtered(rb_state_msgs, vic_msgs):
    t_fil, vx_fil, vy_fil, vyaw_fil = parse_estimator_msgs(rb_state_msgs)
    t_raw, vx_raw, vy_raw, vyaw_raw = parse_vic_msgs(vic_msgs)

    plt.figure()
    plt.subplot(3, 1, 1)
    plt.title('Base velocity estimation')
    plt.plot(t_raw, vx_raw, label='Raw')
    plt.plot(t_fil, vx_fil, label='Filtered')
    plt.ylabel('vx (m/s)')

    plt.subplot(3, 1, 2)
    plt.plot(t_raw, vy_raw, label='Raw')
    plt.plot(t_fil, vy_fil, label='Filtered')
    plt.ylabel('vy (m/s)')

    plt.subplot(3, 1, 3)
    plt.plot(t_raw, vyaw_raw, label='Raw')
    plt.plot(t_fil, vyaw_fil, label='Filtered')
    plt.ylabel('vyaw (rad/s)')
    plt.xlabel('t (s)')
    plt.legend()

    # plt.subplot(3, 1, 2)
    # plt.plot(t, vy)
    #
    # plt.subplot(3, 1, 3)
    # plt.plot(t, vyaw)

    plt.show()


def main():
    if len(sys.argv) < 2:
        print('Must specify bag file.')
        return
    bag = rosbag.Bag(sys.argv[1])
    rb_state_msgs = [msg for _, msg, _ in bag.read_messages('/rb_joint_states')]
    vic_msgs = [msg for _, msg, _ in bag.read_messages('/vicon/ThingBase/ThingBase')]

    # parse_estimator_msgs(rb_state_msgs)
    # parse_vic_msgs(vic_msgs)
    comp_raw_filtered(rb_state_msgs, vic_msgs)


if __name__ == '__main__':
    main()

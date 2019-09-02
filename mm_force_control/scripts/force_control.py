#!/usr/bin/env python2
import rospy
import numpy as np
import tf.transformations as tfs

from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from sensor_msgs.msg import JointState
from mm_msgs.msg import ForceControlState

import mm_kinematics.kinematics as kinematics

import mm_force_control.util as util
from mm_force_control.filter import ExponentialSmoother
from mm_force_control.pid import PID
from mm_force_control.bias import FTBiasEstimator


FORCE_THRESHOLD = 5  # Force required for force control to be used
CONTACT_FORCE = 5  # Desired contact force, when contact is detected
MAX_INPUT_FORCE = 10  # Maximum force value that can input to the PID controller

HZ = 20  # Control loop rate (Hz)

N_BIAS = 100  # Number of samples to use for force bias estimation.

# Controller gains
Kp = np.zeros(3)
Kd = np.zeros(3)
Ki = 0.02 * np.zeros(3)

# Transform from EE frame to force torque sensor frame.
# <node pkg="tf" type="static_transform_publisher" name="FT300_static"
# args="0.02 0.0 0.0 -1.57 0 -1.57 ur10_arm_ee_link
# robotiq_force_torque_frame_id 100" />
f_T_e = tfs.translation_matrix([0.02, 0, 0]).dot(tfs.euler_matrix(-np.pi/2, 0, -np.pi/2))
e_T_f = tfs.inverse_matrix(f_T_e)


class ForceControlNode(object):
    def __init__(self):
        self.pid = PID(Kp, Ki, Kd, set_point=np.zeros(3))
        self.bias = FTBiasEstimator(N_BIAS)

        self.smoother = ExponentialSmoother(tau=0.1, x0=np.zeros(3))

        self.force_raw = np.zeros(3)
        self.force = np.zeros(3)
        self.q = np.zeros(9)
        self.time_prev = rospy.Time.now().to_sec()

        # subs/pubs are initialized last since callbacks are multithreaded and
        # can actually be called before other variables in __init__ have been
        # declared
        self.joint_states_sub = rospy.Subscriber('/mm_joint_states',
                                                 JointState,
                                                 self.joint_states_cb)

        self.force_sub = rospy.Subscriber('/robotiq_force_torque_wrench',
                                          WrenchStamped, self.force_cb)

        self.p_off_pub = rospy.Publisher('/force_control/position_offset',
                                         Vector3Stamped, queue_size=10)

        self.state_pub = rospy.Publisher('/force_control/state',
                                         ForceControlState, queue_size=10)

    def force_cb(self, msg):
        f = msg.wrench.force
        self.force_raw, _ = self.bias.unbias(np.array([f.x, f.y, f.z]),
                                             np.zeros(3))
        now = rospy.Time.now().to_sec()
        dt = now - self.time_prev
        self.time_prev = now

        self.force_filt = self.smoother.next(self.force_raw, dt)

    def joint_states_cb(self, msg):
        # TODO should handle locking at some point
        self.q = msg.position

    def publish_state(self, stamp, force_raw, force_filt, force_world, p_off):
        msg = ForceControlState()
        msg.header.stamp = stamp

        msg.force_raw.x = force_raw[0]
        msg.force_raw.y = force_raw[1]
        msg.force_raw.z = force_raw[2]

        msg.force_filtered.x = force_filt[0]
        msg.force_filtered.y = force_filt[1]
        msg.force_filtered.z = force_filt[2]

        msg.force_world.x = force_world[0]
        msg.force_world.y = force_world[1]
        msg.force_world.z = force_world[2]

        msg.position_offset.x = p_off[0]
        msg.position_offset.y = p_off[1]
        msg.position_offset.z = p_off[2]

        self.state_pub.publish(msg)

    def publish_position_offset(self, stamp, p_off):
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.vector.x = p_off[0]
        msg.vector.y = p_off[1]
        msg.vector.z = p_off[2]
        self.p_off_pub.publish(msg)

    def loop(self, hz):
        rospy.loginfo('Force control started. Estimating FT sensor bias...')
        self.bias.listen()
        rospy.loginfo('Estimated FT bias = {}. Control loop started.'.format(self.bias.bias))

        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
            # Transform force to the world frame
            w_T_e = kinematics.forward(self.q)
            w_T_f = w_T_e.dot(e_T_f)
            f = util.apply_transform(w_T_f, self.force_filt)

            # Force control. Steer toward CONTACT_FORCE if FORCE_THRESHOLD is
            # exceeded.
            comp = np.abs(f) > FORCE_THRESHOLD
            set_point = comp * np.sign(f) * CONTACT_FORCE

            # Bound force input so it can only be between +-FORCE_THRESHOLD
            f_in = util.bound_array(f * comp, -MAX_INPUT_FORCE, MAX_INPUT_FORCE)
            # f_in = f

            p_off = self.pid.update(f_in, set_point=set_point)

            now = rospy.Time.now()
            self.publish_position_offset(now, p_off)
            self.publish_state(now, self.force_raw, self.force_filt, f, p_off)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('mm_force_control_node')

    node = ForceControlNode()
    node.loop(HZ)

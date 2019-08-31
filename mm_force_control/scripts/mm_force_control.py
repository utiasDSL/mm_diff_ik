import rospy
import numpy as np
import tf.transformations as tfs

from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from sensor_msgs.msg import JointState

from mm_kinematics import ThingKinematics
import mm_force_control.util as util
from mm_force_control import PID


FORCE_THRESHOLD = 5  # Force required for force control to be used
CONTACT_FORCE = 5  # Desired contact force, when contact is detected
MAX_INPUT_FORCE = 10  # Maximum force value that can input to the PID controller

DT = 0.1

Kp = np.zeros(3)
Kd = np.zeros(3)
Ki = 0.02 * np.zeros(3)

# Transform from EE frame to force torque sensor frame.
# <node pkg="tf" type="static_transform_publisher" name="FT300_static"
# args="0.02 0.0 0.0 -1.57 0 -1.57 ur10_arm_ee_link
# robotiq_force_torque_frame_id 100" />
f_T_e = tfs.translation_matrix(0.02, 0, 0).dot(tfs.euler_matrix(-np.pi/2, 0, -np.pi/2))
e_T_f = tfs.inverse_matrix(f_T_e)


class ForceSensorBias(object):
    def __init__(self, N):
        self.bias = np.zeros(6)
        self.N = 0
        self.sum = np.zeros(6)
        self.count = 0

        self.ft_sub = rospy.Subscriber('/robotiq_force_torque_wrench',
                                       WrenchStamped, self.ft_cb)

    def ft_cb(self, msg):
        # TODO could unsubscribe when done
        if self.count < self.N:
            data = np.array([msg.wrench.force.x, msg.wrench.force.y,
                             msg.wrench.force.z, msg.wrench.torque.x,
                             msg.wrench.torque.y, msg.wrench.torque.z])
            self.sum += data
            self.count += 1

    def listen(self):
        while self.count < self.N:
            rospy.sleep(0.1)
        self.bias = self.sum / self.N

    def unbias(self, force, torque):
        f = force - self.bias[:3]
        t = torque - self.bias[3:]
        return f, t


class ExponentialSmoother(object):
    def __init__(self, tau, x0):
        self.tau = tau
        self.prev = x0

    def next(self, measured, dt):
        c = 1.0 - np.exp(-dt / self.tau)
        state = c * measured + (1 - c) * self.prev
        self.prev = state
        return state


class ForceControlNode(object):
    def __init__(self):
        self.pid = PID(Kp, Ki, Kd, set_point=np.zeros(3))
        # TODO filter tuning
        self.smoother = ExponentialSmoother(tau=0.1, x0=np.zeros(3))
        self.kin = ThingKinematics()

        self.joint_states_sub = rospy.Subscriber('/mm_joint_states',
                                                 JointState,
                                                 self.joint_states_cb)

        self.force_sub = rospy.Subscriber('/robotiq_force_torque_wrench',
                                          WrenchStamped, self.force_cb)

        self.p_off_pub = rospy.Publisher('/force_position_offset',
                                         Vector3Stamped,
                                         queue_size=10)
        self.state_pub = None  # TODO

        self.force_raw = np.zeros(3)
        self.force = np.zeros(3)

    def force_cb(self, msg):
        f = msg.wrench.force
        self.force_raw, _ = self.bias.unbias(np.array([f.x, f.y, f.z]),
                                             np.zeros(3))
        self.force_filt = self.smoother.filter(self.force_raw)

    def joint_states_cb(self, msg):
        # TODO should handle locking at some point
        self.q = msg.position

    def loop(self, dt):
        while not rospy.is_shutdown():
            # Transform force to the world frame
            # TODO handle homogeneous coordinates
            w_T_e = self.kin.calc_fk(self.q)
            w_T_f = w_T_e.dot(e_T_f)

            # Instead of converting the vectors to homogeneous coords and back,
            # just lop off the last row of the transformation matrix.
            f = w_T_f[:3,:].dot(self.force_filt)

            # Force control. Steer toward CONTACT_FORCE if FORCE_THRESHOLD is
            # exceeded.
            comp = np.abs(f) > FORCE_THRESHOLD
            set_point = comp * np.sign(f) * CONTACT_FORCE

            # Bound force input os it can only be up to +-FORCE_THRESHOLD
            f_in = util.bound_array(f * comp, -MAX_INPUT_FORCE, MAX_INPUT_FORCE)

            p_off = self.force_pid.update(f_in, set_point=set_point)

            # TODO publish p_off
            msg = Vector3Stamped()
            msg.header.stamp = rospy.Time.now()
            msg.vector.x = p_off[0]
            msg.vector.y = p_off[1]
            msg.vector.z = p_off[2]

            self.p_off_pub.publish(msg)

            # TODO publish force state - raw, filtered, world

            rospy.sleep(self.dt)


if __name__ == '__main__':
    rospy.init_node('mm_force_control_node')

    node = ForceControlNode()
    node.loop(DT)

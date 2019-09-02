import rospy
import numpy as np

from geometry_msgs.msg import WrenchStamped


class FTBiasEstimator(object):
    ''' Estimate bias on force-torque sensor. '''
    def __init__(self, N):
        self.N = N
        self.count = 0
        self.bias = np.zeros(6)
        self.sum = np.zeros(6)

        self.ft_sub = rospy.Subscriber('/robotiq_force_torque_wrench',
                                       WrenchStamped, self.ft_cb)

    def ft_cb(self, msg):
        if self.count < self.N:
            data = np.array([msg.wrench.force.x, msg.wrench.force.y,
                             msg.wrench.force.z, msg.wrench.torque.x,
                             msg.wrench.torque.y, msg.wrench.torque.z])
            self.sum += data
            self.count += 1
        else:
            # once we have enough data to calculate the bias, we can
            # unsubscribe from the sensor
            self.ft_sub.unregister()

    def listen(self, dt=0.1):
        ''' Block until the required number of messages have been received. '''
        while self.count < self.N and not rospy.is_shutdown():
            rospy.sleep(dt)
        self.bias = self.sum / self.N

    def unbias(self, force, torque):
        ''' Apply bias correction to force and torque measurements. '''
        f = force - self.bias[:3]
        t = torque - self.bias[3:]
        return f, t

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
        if not self.done():
            data = np.array([msg.wrench.force.x, msg.wrench.force.y,
                             msg.wrench.force.z, msg.wrench.torque.x,
                             msg.wrench.torque.y, msg.wrench.torque.z])
            self.sum += data
            self.count += 1
        else:
            # once we have enough data to calculate the bias, we can
            # unsubscribe from the sensor
            self.ft_sub.unregister()

    def estimate(self, dt=0.1):
        ''' Block until the required number of messages have been received. '''
        rospy.loginfo('Estimating FT sensor bias...')
        while not self.done() and not rospy.is_shutdown():
            rospy.sleep(dt)
        self.bias = self.sum / self.N
        rospy.loginfo('FT bias estimate with {} measurements = {}'.format(self.bias, self.N))

    def done(self):
        return self.count >= self.N

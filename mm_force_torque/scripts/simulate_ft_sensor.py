#!/usr/bin/env python2
"""Generate and publish simulated F/T measurements."""
import rospy
import numpy as np

from geometry_msgs.msg import WrenchStamped


HZ = 100  # Control loop rate (Hz)

MEAN = np.zeros(6)
STDEV = np.array([1.2, 1.2, 0.5, 0.02, 0.02, 0.03])  # from datasheet
VAR = STDEV**2


# Implemented as a class in case we want to do more complicated simulations,
# like with a fake obstacle or a time-varying signal.
class SimulatedFTSensor(object):
    """Simulated Robotiq force/torque sensor."""
    def __init__(self):
        pass

    def measure(self):
        """Get a new measurement."""
        return np.random.normal(loc=MEAN, scale=STDEV)


if __name__ == '__main__':
    rospy.init_node('mm_simulated_ft_node')

    wrench_pub = rospy.Publisher("/robotiq_force_torque_wrench",
                                 WrenchStamped, queue_size=10)

    rate = rospy.Rate(HZ)
    sensor = SimulatedFTSensor()

    while not rospy.is_shutdown():
        wrench = sensor.measure()
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()

        msg.wrench.force.x = wrench[0]
        msg.wrench.force.y = wrench[1]
        msg.wrench.force.z = wrench[2]

        msg.wrench.torque.x = wrench[3]
        msg.wrench.torque.y = wrench[4]
        msg.wrench.torque.z = wrench[5]

        wrench_pub.publish(msg)

        rate.sleep()

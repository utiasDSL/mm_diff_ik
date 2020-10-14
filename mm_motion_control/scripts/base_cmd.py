#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

HZ = 25
DURATION = 5


def main():
    rospy.init_node('base_cmd_node')
    cmd_pub = rospy.Publisher('/ridgeback_velocity_controller/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)

    # velocities
    vx = 0
    vy = 0
    wz = 0

    # accelerations
    ax = 0
    ay = 0
    az = 0

    t0 = rospy.Time.now().to_sec()
    t = t0
    while not rospy.is_shutdown() and t - t0 < DURATION:
        # send the command
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        cmd_pub.publish(msg)

        rate.sleep()

        # update time
        now = rospy.Time.now().to_sec()
        dt = now - t
        t = now

        # integrate acceleration
        if t - t0 <= 0.5*DURATION:
            vx += ax * dt
            vy += ay * dt
            wz += az * dt
        else:
            vx -= ax * dt
            vy -= ay * dt
            wz -= az * dt


if __name__ == '__main__':
    main()

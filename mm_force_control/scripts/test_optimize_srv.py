#!/usr/bin/env python2

import rospy
from rr.srv import Optimize

import IPython


SRV_NAME = 'ik_optimize'


def opt_client():
    pass


def main():
    rospy.wait_for_service(SRV_NAME)
    ik_opt = rospy.ServiceProxy(SRV_NAME, Optimize, persistent=True)

    q_current = [0] * 9
    P_desired = [1, 0, 0, 0, 0, 0]
    dP_desired = [0] * 6

    if ik_opt:
        resp = ik_opt(q_current, P_desired, dP_desired)

    # while not rospy.is_shutdown():
    #     pass

    IPython.embed()


if __name__ == '__main__':
    main()

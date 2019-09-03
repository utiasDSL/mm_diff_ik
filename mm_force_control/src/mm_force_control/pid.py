import rospy
import numpy as np
from mm_force_control.util import bound_array


class PID(object):
    ''' PID controller for force. '''
    def __init__(self, Kp, Ki, Kd, desired=0, decay=0, guard=1000):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.I_term = 0

        self.desired = desired
        self.last_time = rospy.get_time()
        self.last_error = 0

        # saturate the I term at this value
        # use to limit the amount by which the EE is allowed to deviate from
        # the reference trajectory due to force perturbation
        self.windup_guard = guard * np.ones(3)  # 50

        # exponential decay on the I term: I = (1-decay)*I
        # we need this if we want to return to the reference trajectory after
        # force perturbation
        self.decay = decay  # 0.1

    def tick(self):
        ''' Reset recorded time of previous update. Useful for manipulating
            time after initialization but before the control loop has started. '''
        self.last_time = rospy.get_time()

    def update(self, actual, desired=None):
        if desired is not None:
            self.desired = desired

        error = self.desired - actual

        current_time = rospy.get_time()
        delta_time = current_time - self.last_time
        delta_error = error - self.last_error

        # P
        P_term = error

        # I
        self.I_term += error * delta_time
        self.I_term = bound_array(self.I_term, -self.windup_guard,
                                  self.windup_guard)

        # D
        if delta_time > 0:
            D_term = delta_error / delta_time
        else:
            D_term = 0

        self.last_error = error
        self.last_time = current_time

        output = self.Kp * P_term + self.Ki * self.I_term + self.Kd * D_term

        # decay on the I term
        self.I_term -= self.decay * self.I_term

        return output

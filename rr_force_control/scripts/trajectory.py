import copy
import numpy as np


# TODO we're going to want to generalize this to Bezier points


class Trajectory(object):
    ''' Defines a trajectory for the end effector to follow. '''
    def __init__(self, tf0, dt, duration):
        self.tf0 = tf0
        self.tf = copy.deepcopy(tf0)
        self.num_steps = duration / dt
        self.curr_step = 0

    def step(self):
        if self.curr_step >= self.num_steps:
            return False
        self.curr_step += 1
        return True

    def get_tf(self):
        # use a copy so that it can be modified by the caller without changing
        # the underlying state of the trajectory object
        return copy.deepcopy(self.tf)


class LineTrajectory(Trajectory):
    ''' Trajectory that follows a straight line. '''
    def __init__(self, tf0, dt, duration, lx, ly, lz):
        Trajectory.__init__(self, tf0, dt, duration)

        self.lx = lx
        self.ly = ly
        self.lz = lz

    def step(self):
        # If we have reached the end of the trajectory, there is nothing more
        # to be done.
        if not super(LineTrajectory, self).step():
            return False

        self.tf = copy.deepcopy(self.tf0)
        self.tf.translation.x += self.lx * self.curr_step / self.num_steps
        self.tf.translation.y += self.ly * self.curr_step / self.num_steps
        self.tf.translation.z += self.lz * self.curr_step / self.num_steps

        return True


class SineTrajectory(Trajectory):
    ''' Trajectory that follows a sine wave. '''
    def __init__(self, tf0, dt, duration, ly, Rz):
        Trajectory.__init__(self, tf0, dt, duration)

        self.ly = ly
        self.Rz = Rz

    def step(self):
        if not super(SineTrajectory, self).step():
            return False

        angle = 2 * np.pi * self.curr_step / self.num_steps

        self.tf = copy.deepcopy(self.tf0)
        self.tf.translation.y += self.ly * self.curr_step / self.num_steps
        self.tf.translation.z += self.Rz * np.sin(angle)


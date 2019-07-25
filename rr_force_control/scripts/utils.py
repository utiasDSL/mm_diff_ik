from collections import deque
import rospy
import tf2_ros
import numpy as np


def bound_array(a, lb, ub):
    ''' Elementwise bound array above and below. '''
    return np.minimum(np.maximum(a, lb), ub)


class AveragingFilter(object):
    ''' Simple low-pass averaging filter. '''
    def __init__(self, max_size):
        self.max_size = max_size
        self.data = deque()

    def filter(self, value):
        self.data.append(value)
        if len(self.data) > self.max_size:
            self.data.popleft()

        total = 0
        for datum in self.data:
            total += datum
        return total / len(self.data)


class TransformLookup(object):
    ''' Utility to lookup coordinate transforms. '''
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_stamped(self, target_frame, source_frame):
        ''' Lookup the stamped transform between two coordinate frames in the
            TF tree. '''
        try:
            tf_stamped = self.tf_buffer.lookup_transform(target_frame,
                                                         source_frame,
                                                         rospy.Time(0))
            return tf_stamped
        except tf2_ros.TransformException:
            print("TF2 exception occurred.")

    def get(self, target_frame, source_frame):
        ''' Lookup the transform between two coordinate frames in the TF
            tree. '''
        return self.get_stamped(target_frame, source_frame).transform

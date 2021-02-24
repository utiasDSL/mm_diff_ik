import rospy
import tf2_ros
import numpy as np


def apply_transform(T, v):
    ''' Apply 4x4 homogeneous transform T to 3d vector v. '''
    return T.dot(np.append(v, 1))[:3]


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

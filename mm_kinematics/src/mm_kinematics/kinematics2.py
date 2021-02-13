from __future__ import print_function
import os
import numpy as np
import dill

from mm_kinematics import codegen


# possible pickles from which to load functions
FUNCTION_PICKLES = ["kinematics.pkl", "dJdq.pkl"]


class KinematicModel(object):
    """Kinematic model of the Thing mobile manipulator."""

    def __init__(self):
        self._load_generated_functions()

    def _load_generated_functions(self):
        this_dir = os.path.dirname(__file__)

        for pickle in FUNCTION_PICKLES:
            path = codegen.get_default_file_path_from_module(this_dir, pickle)

            try:
                with open(path, "rb") as f:
                    func_dict = dill.load(f)
            except IOError:
                print("Did not find {}. Functions not loaded.".format(pickle))

            self.__dict__.update(func_dict)

    def manipulability(self, q):
        """Calculate manipulability index."""
        J = self.jacobian(q)

        # only the arm Jacobian is relevant for manipulability
        J = J[:, 3:]

        m2 = np.linalg.det(J.dot(J.T))

        # handle numerical errors pushing us slightly negative
        if m2 < 0:
            m2 = 0
        m = np.sqrt(m2)
        return m

    def manipulability_gradient(self, q):
        """Calculate gradient of manipulability index."""
        m_grad = np.zeros(9)

        dJdq_funcs = [self.dJa_dtb, self.dJa_dq1, self.dJa_dq2, self.dJa_dq3,
                      self.dJa_dq4, self.dJa_dq5]

        J = self.jacobian(q)[:, 3:]
        JJT = J.dot(J.T)
        m = np.sqrt(np.linalg.det(JJT))

        # first two and last joint do not affect manipulability
        for i, dJdq_func in enumerate(dJdq_funcs):
            dJdq = dJdq_func(q)
            m_grad[i+2] = m * np.trace(np.linalg.solve(JJT, J.dot(dJdq.T)))

        return m_grad

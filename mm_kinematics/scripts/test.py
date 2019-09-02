import numpy as np
import mm_kinematics.kinematics as kinematics

q = np.zeros(9)
kinematics.forward_chain(q)

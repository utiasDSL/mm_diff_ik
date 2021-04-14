#!/bin/sh
# generate all kinematics functions
python2 ./generate_kinematics_python.py
python2 ./generate_dJdq_python.py
python2 ./generate_jacobian_cpp.py
python2 ./generate_dJdq_cpp.py

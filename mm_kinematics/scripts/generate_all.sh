#!/bin/sh
# generate all kinematics functions
PYTHON_EXECUTABLE=$(command -v python)
if [ -n "$1" ]; then
  PYTHON_EXECUTABLE=$(command -v "$1")
fi
echo "Using Python located at: $PYTHON_EXECUTABLE"
"$PYTHON_EXECUTABLE" ./generate_kinematics_python.py
"$PYTHON_EXECUTABLE" ./generate_dJdq_python.py
"$PYTHON_EXECUTABLE" ./generate_jacobian_cpp.py
"$PYTHON_EXECUTABLE" ./generate_dJdq_cpp.py

# mm_kinematics

Kinematics library for the Thing mobile manipulator. Contains both C++ and
Python implementations.

The "source of truth" is in `src/mm_kinematics/symbolic.py`, which computes the
forward and differential kinematics symbollically. This library is used to
automatically generate components of the other implementations using the
scripts `scripts/generate_*`. You can run all of them using the script
`generate_all.sh`.

# Redundancy Resolution

Redundancy resolution for the Thing mobile manipulator via optimization to
solve the IK problem.

`rr_motion_control` is the `ros_control`-compatible inner-loop motion
controller that implements proportional control followed by an optimization
over the joint velocities. Runs at the speed of the robot controller, 125Hz.
C++.

`rr_force_control` is the outer-loop force controller, also responsible for
trajectory generation. Runs at ~10Hz. Python.

`rr_msgs` contains custom message definitions.

# Mobile Manipulator Code

Redundancy resolution for the Thing mobile manipulator via optimization to
solve the IK problem. More-or-less independent from STARS lab code.

## rr_motion_control

Inner-loop motion controller that implements proportional control with velocity
feedforward in task space, followed by an optimization over the joint
velocities to send to the robot. Runs at the speed of the on-board controller,
125Hz. Written in C++. Optimizer is
[eigen-quadprog](https://github.com/jrl-umi3218/eigen-quadprog).

## rr_force_control

Outer-loop force controller, also responsible for trajectory generation. Runs
at ~10Hz. Written in Python.

## rr_simulation

Basic simulation of the Thing kinematics in Python for visualization. Should
eventually be replaced by a proper Gazebo simulation.

## rr_msgs

Defines custom ROS messages used by other packages in the project.

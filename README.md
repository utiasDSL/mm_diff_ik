# Mobile Manipulator Code

Redundancy resolution for the Thing mobile manipulator via optimization to
solve the IK problem. More-or-less independent from STARS lab code.

## mm_motion_control

Inner-loop motion controller that implements proportional control with velocity
feedforward in task space, followed by an optimization over the joint
velocities to send to the robot. Runs at the speed of the on-board controller,
125Hz. Written in C++. Optimizer is
[eigen-quadprog](https://github.com/jrl-umi3218/eigen-quadprog).

### Subscribers
* `/pose_cmd` (`mm_msgs/PoseTrajectoryPoint`): Takes a trajectory point in task
  space to control toward at a specified future time. Trajectory between
  current position and this point is cubically interpolated. This should be
  published by a higher-level trajectory planner/force controller.
* `/ur10_joint_states` (`sensor_msgs/JointState`): Takes position and velocity
  of the joints of the UR10. Should be published by the UR10 ROS driver
  (`ur_modern_driver`).
* `/rb_joint_states` (`sensor_msgs/JointState`): __TODO__ Takes position and
  velocity of the Ridgeback base; should be published by Vicon.

### Publishers
* `/ur_driver/joint_speed` (`trajectory_msgs/JointTrajectory`): Publishes joint
  speed commands to the UR10. Only the first point is parsed, and then only the
  velocities (and possibly accelerations---not sure about this)
* `/ridgeback_velocity_controller/cmd_vel` (`geometry_msgs/Twist`): Publishes
  joint speed commands to the Ridgeback base.

## mm_force_control

Outer-loop force controller, also responsible for trajectory generation. Runs
at ~10Hz. Written in Python.

## mm_simulation

Basic simulation of the Thing kinematics in Python for visualization. Should
eventually be replaced by a proper Gazebo simulation.

## mm_msgs

Defines custom ROS messages used by other packages in the project.

## mm_vicon

Contains Vicon estimation code and launch files for the Vicon system. To
publish the Thing base pose from Vicon, connect to the Vicon wifi
`DSL_DroneNet_5G` (see the wiki for the password) and run `roslaunch mm_vicon
vicon.launch`. You should see the topic `/vicon/ThingBase/ThingBase` bring
published.

## mm_kinematics

Contains forward kinematics code in both C++ and Python for use by other
packages.

## Running
### Simulation
```bash
> roslaunch mm_motion_control mm_motion_control_sim.launch
> roslaunch mm_simulation sim.launch
> rosrun mm_trajectories launch_traj.py
```

### Experiment
1. Connect to `DSL_DroneNet_5G` network.
2. Connect to the Thing via Ethernet.
3. Run:
```bash
# on laptop
> roslaunch mm_vicon vicon.launch

# on board
> roslaunch mm_motion_control mm_motion_control.launch

# on laptop
> rosrun mm_trajectories launch_traj.py
```

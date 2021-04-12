# Mobile Manipulation

This repository contains ROS packages for control and other utilities for the
UTIAS mobile manipulator (the "Thing"). 

## Contents
1. [Robot Information](#robot-information)
2. [Software Installation](#software-installation)
3. [Packages](#packages)
4. [Simulation](#simulation)
5. [Experiment](#experiment)
6. [Development](#development)

## Robot Information

### Hardware

The robot consists of a UR10 manipulator mounted on a Ridgeback omnidirectional
mobile base. The base has a Hokuyo UST-10LX laser range finder mounted at the
front that provides a two-dimensional scan in a 270 degree arc in front of the
robot. The end effector has a Robotiq FT 300 force torque sensor mounted at the
wrist to measure the applied wrench, as well as Robotiq 3 finger gripper for
manipulation. See the
[datasheets](https://github.com/utiasDSL/dsl__projects__mobile_manipulator/tree/master/datasheets)
for more details on each of the components.

### Software

The robot is currently running Ubuntu 14.04 on its onboard computer, so the
code of this repository is designed to be built and run on that Ubuntu version. 
If you have a later version of Ubuntu, you can emulate a 14.04 environment
using [docker](https://github.com/adamheins/mm-docker).

The UR10 is running firmware version 3.9 and communicating using
[ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver), which
is deprecated. A complete upgrade of the system and dependencies to a more
recent version of Ubuntu, ROS, driver, and firmware is planned for sometime in
the future.

## Software Installation

First, install required dependencies we can get from `apt`:
```
sudo apt install ros-indigo-soem ros-indigo-ur-modern-driver libeigen3-dev
```
Symlink to Eigen:
```
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

Next, clone [vicon_bridge](https://github.com/ethz-asl/vicon_bridge) and
[robotiq](https://github.com/ros-industrial/robotiq) into the `src` directory
of your catkin workspace. 

Finally, you'll also need to install [qpOASES](https://github.com/coin-or/qpOASES).
Clone the repository somewhere convenient (not your catkin workspace; it's not
a ROS package). Then, to compile, simply enter the cloned repo and type `make`.
To get ROS to find the qpOASES headers, you'll need to make some symlinks:
```
sudo ln -s <path/to/qpOASES>/include/qpOASES.hpp /usr/local/include/qpOASES.hpp
sudo ln -s <path/to/qpOASES>/include/qpOASES     /usr/local/include/qpOASES
sudo ln -s <path/to/qpOASES>/bin/libqpOASES.so   /usr/local/lib/libqpOASES.so
```

Now, clone this repository into the `src` directory of your catkin workspace.
Then you should be able to build with `catkin`:
```bash
# normal build
catkin build

# with compiler optimizations (this makes it run much faster)
catkin build -DCMAKE_BUILD_TYPE=Release
```

You'll need some Python packages:
```
pip2 install --user numpy sympy dill
```

Generate additional kinematic functions (these are computed using symbolic math
in Python and then fast Python and C++ code is generated for them). Run all of
the scripts starting with `generate_` in `mm_kinematics/scripts`. You'll need
to compile again afterward: go ahead and run `catkin build` again.

## Packages

Within this metapackage, we have:

### mm_control

Controllers for both joint space and Cartesian (task) space. Written in C++.


### mm_force_torque

Tools for working with the Robotiq force-torque sensor.

To run the driver for real sensor, use
```
roslaunch mm_force_torque bringup.launch
```
but note that **this must be run onboard the robot.** A very basic simulation
of the FT sensor is also available in `scripts/simulate_ft_sensor.py`.

The main utility is the script `scripts/wrench.py`, which processes wrenches
from the sensor by filtering and rotating into the world frame. This
information is then published under the topic `/mm_wrench/info`.


### mm_gripper

Very basic tools for working with the gripper.

Bring up the gripper using
```
roslaunch mm_gripper bringup.launch
```
Then command the gripper to open or close using
```
rosrun mm_gripper gripper.py <cmd> [delay]
```
where `<cmd>` is either `o` for open or `c` for close, and `[delay]` is a an
optional number of seconds to wait before executing the desired command. For
example,
```
rosrun mm_gripper gripper.py c 5
```
will close the gripper in 5 seconds, which allows me to position something to
be grabbed.


### mm_kinematics

Contains forward kinematics code in both C++ and Python for use by other
packages.


### mm_lidar

Tools for using the Hokuyo laser scanner mounted at the front of the base. The
laser scanner is automatically started by the software onboard the robot and
publishes its raw data to `/front/scan`. This package contains scripts for
processing the detections into actual positions of objects in the world frame,
primarily for obstacle avoidance.


### mm_math_util

Miscellaneous math utilities shared by other packages.


### mm_msgs

Defines custom ROS messages used by other packages in the project.


### mm_simulation

Basic simulation of the Thing kinematics in Python with a simple matplotlib
visualization. There is also a Gazebo simulation available
[here](https://github.com/utiasDSL/dsl__projects__mobile_manipulator_gazebo).


### mm_trajectories

Trajectory generation. Generates trajectories to be tracked by the controllers.


### mm_vicon

Contains Vicon estimation code and launch files for the Vicon system. To
publish the mobile base pose from Vicon, connect to the Vicon wifi
`DSL_DroneNet_5G` (see the wiki for the password) and run `roslaunch mm_vicon
vicon.launch`.


## Simulation

This repository contains a very basic simulation of the robot in
`mm_simulation`, which is just a wireframe in a 3D matplotlib plot. To use
this, run:
```bash
> roscore
> roslaunch mm_simulation simulation.launch
```

Alternatively, there is a separate
[respository](https://github.com/utiasDSL/dsl__projects__mobile_manipulator_gazebo)
containing packages for a Gazebo simulation, which should be entirely
compatible with the controllers in this package. Assuming that repository has been installed in the workspace, built, and sourced, then instead of
```
> roslaunch mm_simulation sim.launch
```
one can use
```
> roslaunch mm_gazebo simulation.launch
```
Everything else should remain the same.

## Experiment
Steps for experiments on the real robot:

1. Connect to `DSL_DroneNet_5G` network.
2. Connect to the Thing via Ethernet.
3. Ensure ROS is configured to use the Thing as master:
   ```bash
   laptop > export ROS_IP=192.168.131.100
   laptop > export ROS_MASTER_URI=http://cpr-tor11-01:11311
   ```
   If you run `rostopic list`, you should see various topics related to the Thing
   listed.
4. Run:
   ```bash
   # on laptop
   laptop > roslaunch mm_vicon vicon.launch

   # alternatively, if Vicon is not available
   laptop > roslaunch mm_vicon spoof.launch

   # start the UR10 driver (the Ridgeback starts automatically onboard the
   # robot) and the mux node that publishes the combined /mm_joint_states
   # topic.
   # NOTE: this does usually run fine on the laptop, but can occasionally
   # generate garbage values for joint position or velocity. On the other hand,
   # running on the Thing may have time sync issues between the two computers.
   laptop > roslaunch mm_control bringup.launch

   # start the controller, e.g. differential inverse kinematics
   laptop > rosrun mm_control diff_ik

   # if using force control
   robot > roslaunch mm_force_control force.launch
   laptop > rosrun mm_force_control force.py

   # example of running a trajectory
   # inspect the file first to ensure the trajectory does what you expect
   laptop > rosrun mm_trajectories line.py
   ```

### Go Home
 Requires `bringup.launch` to already be launched and running. Run:
```bash
laptop > roslaunch mm_control home.launch config:=<config>
```
where `<config>` the name of one of the home configurations defined in
`mm_control/config/home.yaml`. If the `config` argument is omitted, it defaults
to the `standard` home position.

### Gripper
Open or close the gripper (experiment only). Run on the laptop:
```bash
laptop > roslaunch mm_gripper bringup.launch

# <cmd> is either o (open) or c (close)
laptop > rosrun mm_gripper gripper.py <cmd>
```

### Set desired force
To set the desired force at runtime, use:
```
laptop > rostopic pub /force/desired std_msgs/Float64 <value>
```

## Development

C++ code should be formatted using
[clang-format](https://clang.llvm.org/docs/ClangFormat.html) and Python code
should be formatted using [Black](https://github.com/psf/black).

C++ tests use [gtest](https://github.com/google/googletest) and Python tests
use [pytest](https://docs.pytest.org/en/stable/index.html).

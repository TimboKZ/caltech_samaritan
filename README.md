# Caltech Samaritan

Autonomous SLAM & exploration using drones, a project for Caltech's *ME 134
Autonomy* class. This repository can be cloned as a ROS package.

# Demo

> Coming soon.

# Prerequisites

This project was developed for ROS Kinetic (Ubuntu 16.04). The following
packages are required:

1. Drone simulation is done using
   [hector\_quadrotor](http://wiki.ros.org/hector_quadrotor) and Gazebo. [This
   page](https://answers.ros.org/question/244776/is-it-possible-to-run-the-hector_quadrotor-demos-in-kinetic/)
   contains the instructions for installing `hector_quadrotor` on ROS Kinetic.

2. [OctoMap](http://wiki.ros.org/octomap) is used to generate the 3D occupancy
   grid. `octomap_server` is used to interface OctoMap with ROS, both can be
   installed via `apt` as `ros-kinetic-octomap` and `ros-kinetic-octomap-ros`.

3. You'll need to install the [OctoMap RViz
   plugin](https://github.com/OctoMap/octomap_rviz_plugins) via `apt` as
   `ros-kinetic-octomap-rviz-plugins`. Without it, occupancy grid
   visualisations in RViz will not work.

3. To control the drone manually, you might need to install the
   `ros-kinetic-teleop-twist-keyboard` package and then run
   `teleop_twist_keyboard teleop_twist_keyboard.py`.
   
4. MoveIt is used for navigation, it can be installed via `apt` as
   `ros-kinetic-moveit` and `ros-kinetic-moveit-plugins`. Among other things, this will install an RViz plugin
   that is used to visualise MoveIt joints on the quadrotor.
   
4. The scripts in this repo are written in Python 2.7 (default version for ROS
   Kinetic).

# Usage

1. Make sure you have the [prerequisites](#prerequisites) installed.
2. Clone this repo into your [catkin
   workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), e.g.
   into `~/catkin_ws/src/caltech_samaritan/`.
3. Source appropriate ROS files in your Bash instance, i.e. run `source
   /opt/ros/kinetic/setup.bash` and `source ~/catkin_ws/devel/setup.bash`.
4. Run `catkin_make` in `~/catkin_ws/` and `source
   ~/catkin_ws/devel/setup.bash` again.
5. Start the simulation using `roslaunch caltech_samaritan
   gazebo_depth_octomap.launch`. This launch file will:
   1. Initialize Gazebo with a sample enivronment,
   2. Spawn a simulated quadrotor with a Kinect attached to it and activate its
      motors,
   3. Launch RViz with a custom config, and
   4. Start an OctoMap server, generating occupancy grid in real time.
6. ???
7. Profit!

# WallFollow

Code for navigation around bridge structures using routines for flight alongside the bridge, flight along a bridge column, etc.

Note: The vert_dev branch serves as the current base branch (master is not the base branch!)
The branches vert_pc (simulation) and vert_eagle (actual UAV) are essentially the same as vert_dev with a couple of parameter values different as one branch is used on the simulator computer and the other on the NVIDIA Jetson onboard the UAV.


## Pre-requisites
* ROS Kinetic + Gazebo 7
* Gazebo
* Mavros: ROS node to communicate with PX4 firmware
* PX4 firmware : code for low level UAV flight control (on actual Pixhawk hardware or as software-in-the-loop (SITL) for Gazebo simulation)

The PX4 website contains information and instructions for installing PX4 firmware and Mavros. https://dev.px4.io/en/setup/dev_env_linux.html

The ubuntu_sim_ros_gazebo.sh script from the PX4 website may help install all the above pre-reqs conveniently. Works well on a clean Ubuntu installation on most conventional PCs. Have had issues getting it to work on NVIDIA Jetson, in which case manual installation of the pre-reqs had to be done.

Refer to this document for help regarding PX4 and Mavros and some clarity on software architecture: https://docs.google.com/document/d/1i-lWuEwOOzB-5ZloPOaRV5dGP6J1aeJemh8ISfB3lxE/edit?usp=sharing (Access restricted to those within Virginia Tech)

## Dependencies
* Catkin_tools: http://catkin-tools.readthedocs.io/en/latest/installing.html
* Gazebo models and worlds from this repo: 
* PCL
* Sweep ROS wrapper

## Steps:
1. Convenience script
2. mavros_px4 help: .bashrc
3. Make posix sitl gazebo
4. Source devel/setup.bash
5. Simulation:  copy and replace the models and worlds folders under your ~/src/Firmware/Tools/sitl_gazebo with those from the sitl_gazebo_wall_follow_sim repository
 edit ~/src/Firmware/launch/mavros_posix_sitl.launch for correct WORLD: <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/2_wall_new.world"/>

## Launch files
1. Flight beside bridge girder
Simulation: $ roslaunch wall_follow besideWall_simulation.launch
Real World: $ roslaunch wall_follow besideWall_real.launch 

2. Flight along a bridge column
Simulation: 
Simulation: $ roslaunch wall_follow columnLoop_simulation.launch
Real World: $ roslaunch wall_follow columnLoop_real.launch 

## Nodes
### /hor/pc_subscriber

Horizontal point cloud subscriber: 
* Processes the horizontal lidar data (filtering + Hough Transform) 
* Publishes line(s) that estimate bridge structure

Subscribed topic:

* */hor/pc2* (sensor_msgs/PointCloud2)

Published topic:

* */hor/ho/li* (wall_follow/Lines)

### vert/pc_subscriber

Vertical point cloud subscriber: 
* Processes the vertical lidar data (filtering + Hough Transform)
* Publishes line(s) that estimate bridge structure

Subscribed topic:

* */vert/pc2* (sensor_msgs/PointCloud2)

Published topic:

* */vert/ho/li* (wall_follow/Lines)

### /besideWall

Flight alongside the bridge girder:
 * Maintains desired distance perpendicular to the girder
 * Maintains desired altitude w.r.t the top of the girder
 * Contanst fixed velocity parallel to the girder
 
Subscribed topics:

* */hor/ho/li* (wall_follow/Lines)
* */vert/ho/li* (wall_follow/Lines)

Published topic:

* */mavros/setpoint_raw/local* (mavros_msgs/PositionTarget)



## Parameters:


## Supporting documentation
* ICRA paper:
* ISER paper:
* MEng slides:

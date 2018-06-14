# WallFollow

This wall_follow package contains code for navigation around bridge structures using routines for flight alongside the bridge, flight along a bridge column, etc. 


## Pre-requisites
* ROS Kinetic + Gazebo 7
* Mavros: ROS node to communicate with PX4 firmware
* PX4 firmware : code for low level UAV flight control (on actual Pixhawk hardware or as software-in-the-loop (SITL) for Gazebo simulation)

The PX4 website contains information and instructions for installing PX4 firmware and Mavros. https://dev.px4.io/en/setup/dev_env_linux.html

The ubuntu_sim_ros_gazebo.sh script from the PX4 website may help install all the above pre-reqs conveniently. Works well on a clean Ubuntu installation on most conventional PCs. Have had issues getting it to work on NVIDIA Jetson, in which case manual installation of the pre-reqs had to be done.

*Refer to this document for help regarding PX4 and Mavros and some clarity on software architecture*: https://docs.google.com/document/d/1i-lWuEwOOzB-5ZloPOaRV5dGP6J1aeJemh8ISfB3lxE/edit?usp=sharing (Access restricted to those within Virginia Tech)

## Dependencies
* Catkin_tools: http://catkin-tools.readthedocs.io/en/latest/installing.html (other catkin build tool will also likely work)
* PCL: http://wiki.ros.org/pcl (It should be included in the standard ROS installation)
* Scanse Sweep ROS driver: https://github.com/scanse/sweep-ros.git (Reads data from the Scanse Sweep LIDAR and publishes it as a point cloud; not required for simulation)
* Gazebo models and worlds from this repository: https://github.com/raaslab/sitl_gazebo_wall_follow.git (only required for simulation)

Copy and replace the models and worlds folders under your `~/src/Firmware/Tools/sitl_gazebo` with those from the sitl_gazebo_wall_follow repository

Edit ~/src/Firmware/launch/mavros_posix_sitl.launch and ensure the below line to launch the correct WORLD: 

`<arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/2_wall_new.world"/>`

## Launch files
### Flight beside bridge girder
Simulation: `roslaunch wall_follow besideWall_simulation.launch`

Real World: `roslaunch wall_follow besideWall_real.launch` 

### Flight along a bridge column
Simulation: `roslaunch wall_follow columnLoop_simulation.launch`

Real World: `roslaunch wall_follow columnLoop_real.launch` 

## Nodes
### /hor/pc_subscriber

Horizontal point cloud subscriber: 
* Processes the horizontal lidar data (filtering + Hough Transform) 
* Publishes line(s) that estimate bridge structure

Subscribed topic:

* */hor/pc2* (sensor_msgs/PointCloud2)

Published topic:

* */hor/ho/li* (wall_follow/Lines)

### /vert/pc_subscriber

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

### /columnLoop

Flight along a bridge column:
 * Maintains desired distance to the column 
 * Maintains centering w.r.t the column
 * Contanst fixed velocity in the vertical direction
 
Subscribed topics:

* */hor/ho/li* (wall_follow/Lines)
* */vert/ho/li* (wall_follow/Lines)

Published topic:

* */mavros/setpoint_raw/local* (mavros_msgs/PositionTarget)



## Parameters:


## Supporting documentation
* ICRA paper:
* ISER paper:
* Video: 

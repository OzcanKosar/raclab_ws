
# Odometry, IMU, Kalman Filter, Extented Kalman Filter, Sensor Fusion

![](https://raclab.org/wp-content/uploads/2021/07/RACLAB5-1-1024x181.png)



## Getting Started
## Installation

[ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) is used during development.
- Installation ros package.

``` bash 
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-ros-controllers ros-noetic-twist-mux
```
``` bash
sudo apt install snapd
```
``` bash
sudo apt-get install ros-noetic-turtlebot3-*
```
``` bash
sudo apt install ros-noetic-robot-localization
```
``` bash
sudo apt install ros-noetic-tf2-tools 
```
``` bash
sudo apt install  ros-noetic-plotjuggler-ros
```
###  Installation raclab_ws
- In your home directory

``` bash
git clone https://github.com/OzcanKosar/raclab_ws.git
```
``` bash
cd raclab_ws
```
``` bash
catkin_make
```
- Add the following to the .bashrc file.

``` bash
export TURTLEBOT3_MODEL=waffle
source ~/raclab_ws/devel/setup.bash
```


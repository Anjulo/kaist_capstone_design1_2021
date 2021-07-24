# KAIST ME Capstone1 2021 Spring Team B

KAIST 2021 Spring Capstone Design 1, Team B

This is a repository for capstone design 1 Team B codes and final presentation. 

## Mission

In this Capstone Design, we designed and built a mobile robot system that can accomplish the given mission autonomously with no human intervention. The mobile robot system includes a vehicle platform equipped with actuators (for mobility and interactions) and sensors (for perception and navigation). The system also includes computer algorithms for vehicle autonomy including sensor data processing, automatic control and associated decision making. We designed and developed the system in a virtual simulation environment using software tools such as SolidWorks, Coppeliasim and ROS with OpenCV.
- Check the [documents](https://github.com/Anjulo/kaist_capstone_design1_2021/tree/main/documents) to read more about the mission problem and presentation of our final solution.

## Requirements

- Required Ubuntu 20.04
- Install ROS Melodic ([official ros website](http://wiki.ros.org/noetic/Installation/Ubuntu))
-Install additional dependencies

```console
 sudo apt install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control 
```
- Restart your machine

```console
sudo reboot 
```

- Move to the workspace

 ```console
 cd ~/catkin_ws/src
 ```

## Download

- Download the repo

 ```console
git clone https://github.com/Anjulo/kaist_capstone_design1_2021.git
```

- Build the packages

 ```console
 catkin build 
 ```
## Launch

- Run the main mission 

```console
roslaunch data_integrate main_mission.launch
```

- Run the bonus mission 

```console
roslaunch data_integrate bonus_mission.launch
```
 
## lane tracing

Node for detecting and tracinga lane

It publishes the information of lane position with respect to the robot helps it navigate in the entrance region, in addition to the lidar localization and maping. 
#### Usage

The nodes needed to run for lane tracing are:

```console
rosrun lane_detection lane_detect_node
rosrun depth_node depth_node
rosrun data_integrate line_trace_node
```


## Ball Harvesting

Node for detecting red and yellow balls.

It publishes the information of ball position and color by calculating geometry between the robot and balls.

#### Usage

The nodes needed to run for Ball Harvesting are:

```console
rosrun ball_detection ball_detect_node
rosrun data_integrate data_integration_node
```

## coppeliasim_models

The map models(.ttt) and  robot & sensor models(.ttm) are [provided](https://github.com/anjulo/Capstone1_2021Spring) by KAIST ME405 TAs. (map_ver_x.ttt, bonus_map.ttt, my_robot_ver_x.ttm files)

We have designed our own robot. (B_Final.ttm)

Just drag and drop files to coppeliasim window.


## core_msgs

A package for defining custom messages used in all codes.

ex) ball_position.msg



## data_integrate

Subscribing lidar(laser_scan) and camera data and do something


#### Usage

```console
# data integrate
rosrun data_integrate data_integrate_node
# data show
rosrun data_integrate data_show_node
```

## robot_teleop

Nodes for manually manipulating the gripper and robot wheels.

#### Usage

```console
# gripper
rosrun robot_teleop prismatic_teleop_key
# wheel
rosrun robot_teleop wheel_teleop_key
```



## Members

1.  [Bereket Yisehak](http://github.com/Anjulo/)
2.  Jeong WooJin
3.  Jang SeungHeun
4.  Jeong SeoHee
5.  Kim GaHyun
6.  Choi YoungSuk
7.  Sabuhi Mikayilov

# KAIST ME Capstone1 2021 Spring Team B

KAIST 2021 Spring Capstone Design 1

This is a repository for capstone design 1 Team B codes. Check the [documents](.../documents) to know about the mission problem and presentation of our final solution. 
To download the package, write the command below in ~/catkin_ws/src terminal.
```console
git clone https://github.com/Anjulo/kaist_capstone_design1_2021.git
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

The map models(.ttt) and  robot & sensor models(.ttm) are provided.

map_ver_x.ttt, bonus_map.ttt, my_robot_ver_x.ttm files

Just drag and drop files to coppeliasim window.

For reference, urdf file for my_robot is uploaded.

1. mylidar_hokuyo.ttm

- ROS

frame : base_scan

topic : /scan

msg type : sensor_msgs::LaserScan

- Lidar specs

rate : 5Hz

resolution : 1 deg -> 360 points per 1 scan

min range : 0.12 m

max range : 3.5 m


2. myimu.ttm

- ROS

frame : imu

topic : /imu

msg type : sensor_msgs::Imu


3. mykinect.ttm

- Camera parameters

FOV (field of view) : 57 deg

W : 640 pixels

H : 480 pixels

f : 589.37 pixels


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


## Tips

- If the prismatic joint fall down, check "Lock motor when target velocity is zero" in Joint Dynamic Properties in CoppeliaSim.
- Click "Toggle real-time mode" when you test your algorithms in simulator.
- ...



## Members

    1.  Bereket Yisehak
	2.  Jeong WooJin
	3.  Jang SeungHeun
	4.  Jeong SeoHee
	5.  Kim GaHyun
    6.  Choi YoungSuk
	7.  Sabuhi Mikayilov

#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>
#include <time.h>

#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"
#include "std_msgs/Float32MultiArray.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>

#include "opencv2/opencv.hpp"

boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400] = {
	999,
};
float lidar_obs;
float min_lidar_dist = 99999;
float min_x = 99999;
float min_y = 99999;

//ball
int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];
float ball_angle[20];

int red_count;
int green_count;

float goal_distance = 100; //dummy data
float goal_angle = 5;	   //dumy data
float min_ball_angle = 10;
float ball_dist = 100; //dummy data

float simtime;
int action;

int len;
int n;

int flag;

#define RAD2DEG(x) ((x)*180. / M_PI)
#define DEG2RAD(x) ((x)*M_PI / 180)

//velocity
int max_vel = DEG2RAD(400);
int min_vel = DEG2RAD(100);
int min_ang_vel = DEG2RAD(125);
int max_ang_vel = DEG2RAD(360);


std::vector<float> lidar_x;
std::vector<float> lidar_y;
std::vector<int> wall_direction;

void time_Callback(const std_msgs::Float32::ConstPtr &msg)
{
	simtime = msg->data;
}
void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	// i==0 : back(-180 degree)
	// i==90 : right(-90 degree)
	// i==180 : front(0 degree)
	// i==270 : left(90 degree)
	map_mutex.lock();

	int count = (scan->angle_max - scan->angle_min) / scan->angle_increment;
	lidar_size = count;
	for (int i = 0; i < count; i++)
	{
		lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
		lidar_distance[i] = scan->ranges[i];
	}
	map_mutex.unlock();
}

void camera_Callback(const core_msgs::ball_position::ConstPtr &position)
{
	float min_distance = 99999;
	red_count = position->size_red;
	green_count = position->size_green;
	for (int i = 0; i < red_count; i++)
	{
		ball_distance[i] = position->distance_red[i];
		ball_angle[i] = position->theta_red[i];

		if (position->distance_red[i] <= min_distance)
		{ //renew the minimum value
			min_distance = position->distance_red[i];
			ball_dist = position->distance_red[i];
			min_ball_angle = RAD2DEG(position->theta_red[i]);
			std::cout << position->distance_red[i] << "       ";
			std::cout << ball_dist << "      ";
			std::cout << position->theta_red[i] << "      ";
			std::cout << min_ball_angle << std::endl;
		}
	}

	for (int i = 0; i < green_count; i++)
	{
		goal_distance = position->distance_green[i];
		goal_angle = position->theta_green[i];
	}
}
int no_obstacle(){
	ros::spinOnce;
	int safe = 1;
	int left = 0;
	int mid = 0;
	int right = 0;
	float min_left = 999;
	float min_right = 999;
	int diff;
	float range1, range2;
	diff = 80;
	range1 = 0.4;
	range2 = 0.5;

	for (int i = 180-diff; i < 180; i++)
	{
		if (i < 170)
		{
			if (lidar_distance[i] < range1)
			{
				std::cout << "lidar distance " << i << " : " << lidar_distance[i] << std::endl;
				safe = 0;
				right = 1;
			}
		}
		else
		{
			if (lidar_distance[i] < range2)
			{
				std::cout << "lidar distance " << i << " : " << lidar_distance[i] << std::endl;
				safe = 0;
				mid = 1;
			}
		}
	}
	for (int i = 180; i < 180+diff; i++)
	{
		if (i < 190)
		{
			if (lidar_distance[i] < range2){
				std::cout << "lidar distance " << i << " : " << lidar_distance[i] << std::endl;
				safe = 0;
				mid = 1;
			}
		}
		else{
			if (lidar_distance[i] < range1)
			{
				std::cout << "lidar distance " << i << " : " << lidar_distance[i] << std::endl;
				safe = 0;
				left = 1;
			}
		}
	}
	if (safe == 1)
		return 1; // no obstacle, goto_goal zone

	if (left == 1 && mid == 1 && right == 1)
		return -2; //should move backward
	for (int i = 50; i < 130; i++){
		if (lidar_distance[i] < min_right){
			min_right = lidar_distance[i];
		}
	}
	for (int i = 230; i < 310; i++)
	{
		if (lidar_distance[i] < min_left){
			min_left = lidar_distance[i];
		}
	}
	if (min_right > min_left){
		if(left==1) return -1; //turn right
		else if(min_left > 0.6 && right == 1 && left ==0) return 0; //turn left
		else return -1;
	}
	else if(min_right <= min_left){
		if(right==1) return 0;
		else if(min_right >0.6 && left==1 && right ==0) return -1;
		else return 0;
	}
	return -2;
}

int harvest_available()
{
	ros::spinOnce;
	int safe = 1;
	int left = 0;
	int mid = 0;
	int right = 0;
	float min_left = 999;
	float min_right = 999;
	int diff;
	float range1, range2;
	if(ball_dist > 1){
		diff = 10;
		range1 = 0.3;
		range2 = 0.1;
	}
	else{
		diff = 60;
		range1 = 0.1;
		range2 = 0.1;
	}

	for (int i = 180-diff; i < 180; i++){
		if (i < 175){
			if (ball_dist > lidar_distance[i] - range1){
				std::cout << "lidar distance " << i << " : " << lidar_distance[i] << std::endl;
				safe = 0;
				right = 1;
			}
		}
		else{
			if (ball_dist > lidar_distance[i] - range2){
				std::cout << "lidar distance " << i << " : " << lidar_distance[i] << std::endl;
				safe = 0;
				mid = 1;
			}
		}
	}
	for (int i = 180; i < 180+diff; i++){
		if (i < 190){
			if (ball_dist > lidar_distance[i] - range2){
				std::cout << "lidar distance " << i << " : " << lidar_distance[i] << std::endl;
				safe = 0;
				mid = 1;
			}
		}
		else{
			if (ball_dist > lidar_distance[i] - range1){
				std::cout << "lidar distance " << i << " : " << lidar_distance[i] << std::endl;
				safe = 0;
				left = 1;
			}
		}
	}

	if (safe == 1)
		return 1; //harvest available

	if (left == 1 && mid == 1 && right == 1)
		return -2; //should move backward
	for (int i = 70; i < 110; i++){
		if (lidar_distance[i] < min_right)
		{
			min_right = lidar_distance[i];
		}
	}
	for (int i = 250; i < 290; i++)
	{
		if (lidar_distance[i] < min_left)
		{
			min_left = lidar_distance[i];
		}
	}
	if (min_right > min_left){
		if(min_left > 0.5){
			if(right==1) return 0;
			else return -1;
		}
		return -1;
	}
	else if(min_right <= min_left){
		if(min_right > 0.5){
			if(left==1) return -1;
			else return 0;
		}
	}
	return -2;
}

void open_gripper(ros::Publisher joint)
{
	std::cout << "open the gripper" << std::endl;
	std_msgs::Float64 joint_command;
	joint_command.data = 0; //should be fixed
	joint.publish(joint_command);
	ros::Duration(3).sleep();
}

void close_gripper(ros::Publisher gripper_joint)
{
	std::cout << "close the gripper" << std::endl;
	std_msgs::Float64 joint_command;
	joint_command.data = -0.1;
	gripper_joint.publish(joint_command);
	ros::Duration(3).sleep();
}

void gripper_up(ros::Publisher el_gripper)
{
	std::cout << "gripper up" << std::endl;
	std_msgs::Float64 joint_command;
	joint_command.data = 0.25;
	el_gripper.publish(joint_command);
	ros::Duration(3).sleep();
}

void gripper_down(ros::Publisher el_gripper)
{
	std::cout << "gripper down" << std::endl;
	std_msgs::Float64 joint_command;
	joint_command.data = 0;
	el_gripper.publish(joint_command);
	ros::Duration(3).sleep();
}

void stop(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data = 0;
	right_wheel_msg.data = 0;
	left_wheel.publish(left_wheel_msg);
	right_wheel.publish(right_wheel_msg);
	ros::Duration(0.5).sleep();
	std::cout << "stop process is done!!!" << std::endl;
}

//move backward after the goal
void move_back(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data = -max_vel;
	right_wheel_msg.data = -max_vel;
	left_wheel.publish(left_wheel_msg);
	right_wheel.publish(right_wheel_msg);
	ros::Duration(4).sleep();
	stop(left_wheel, right_wheel);
}

void align(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;

	std::cout << "align function is called" << std::endl;
	int count = 0;
	while (ros::ok())
	{
		count++;
		double end = ros::Time::now().toSec();
		ros::spinOnce();
		if (lidar_distance[180]<0.4 && count > 500) {
			std::cout << "too much rotate, find the ball" << std::endl;
			move_back(left_wheel, right_wheel);
			}
		
		if (min_ball_angle > -0.1 && min_ball_angle < 0.1)
		{
			std::cout << min_ball_angle << std::endl;
			break;
		}
		std::cout << "loop is called" << std::endl;
		if (min_ball_angle < 0)
		{ //rotate clockwise direction
			std::cout << "rotate CW" << std::endl;
			if (min_ball_angle > -5)
			{
				left_wheel_msg.data = DEG2RAD(50);
				right_wheel_msg.data = -DEG2RAD(50);
			}
			else
			{
				left_wheel_msg.data = DEG2RAD(100);
				right_wheel_msg.data = -DEG2RAD(100);
			}
		}
		else
		{ //rotate counter-clockwise direction s
			std::cout << "rotate CCW" << std::endl;
			if (min_ball_angle < 5)
			{
				left_wheel_msg.data = -DEG2RAD(50);
				right_wheel_msg.data = DEG2RAD(50);
			}
			else
			{
				left_wheel_msg.data = -DEG2RAD(100);
				right_wheel_msg.data = DEG2RAD(100);
			}
		}
		left_wheel.publish(left_wheel_msg);
		right_wheel.publish(right_wheel_msg);
	}
	stop(left_wheel, right_wheel);
	ros::Duration(0.5).sleep();
}

//lift the legs(l2, r2, l4, r4)
void legs_lift(ros::Publisher leg)
{
	std_msgs::Float64 leg_msg;
	leg_msg.data = -0.01;
	leg.publish(leg_msg);
	std::cout << "legs_lift function is called" << std::endl;
}

//initialize the settings
void stat_init(ros::Publisher left_wheel, ros::Publisher right_wheel, ros::Publisher leg, ros::Publisher gripper, ros::Publisher el_gripper)
{
	std::cout << "stat_init function is called " << std::endl;
	stop(left_wheel, right_wheel);
	legs_lift(leg);
	open_gripper(gripper);
	gripper_down(el_gripper);
}
void move(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data = max_vel;
	right_wheel_msg.data = max_vel;
	while (min_lidar_dist > 1)
	{
		left_wheel.publish(left_wheel_msg);
		right_wheel.publish(right_wheel_msg);
	}
	stop(left_wheel, right_wheel);
}

void go_to_ball(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	ros::spinOnce();
	if (harvest_available() == 1)
	{
		std::cout << "no obstacle " << ball_dist << std::endl;
		while (ros::ok())
		{
			ros::spinOnce();
			if (red_count == 0)
			{
				align(left_wheel, right_wheel);
			}
			std::cout << "red_ball_dist : " << ball_dist << std::endl;
			if (ball_dist < 0.8)
			{
				stop(left_wheel, right_wheel);
				align(left_wheel, right_wheel);
				ros::Duration(0.5).sleep();
				break;
			}
			left_wheel_msg.data = max_vel;
			right_wheel_msg.data = max_vel;
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);
		}
		while (ros::ok())
		{
			ros::spinOnce();
			std::cout << "ball_dist(<0.8) : " << ball_dist << std::endl;
			left_wheel_msg.data = max_vel;
			right_wheel_msg.data = max_vel;
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);

			if (ball_dist < 0.22)
			{
				std::cout << "ball dist less than 0.22" << std::endl;
				ros::Duration(0.8).sleep();
				stop(left_wheel, right_wheel);
				break;
			}
			else if (ball_dist < 0.35)
			{
				std::cout << "ball dist less than 0.4" << std::endl;
				stop(left_wheel, right_wheel);
				align(left_wheel, right_wheel);
				left_wheel_msg.data = max_vel;
				right_wheel_msg.data = max_vel;
				left_wheel.publish(left_wheel_msg);
				right_wheel.publish(right_wheel_msg);
				ros::Duration(1.6).sleep();
				stop(left_wheel, right_wheel);
				break;
			}
		}
	}
	if (harvest_available() == -1) //should turn right
	{							   // pole is left
		std::cout << "the obstacle is on left" << std::endl;
		left_wheel_msg.data = DEG2RAD(100); //rotate CW
		right_wheel_msg.data = -DEG2RAD(100);
		left_wheel.publish(left_wheel_msg);
		right_wheel.publish(right_wheel_msg);
		ros::Duration(2).sleep();
		
		left_wheel_msg.data = DEG2RAD(200); //go forward
		right_wheel_msg.data = DEG2RAD(200);
		left_wheel.publish(left_wheel_msg);
		right_wheel.publish(right_wheel_msg);
		ros::Duration(4).sleep();
		align(left_wheel, right_wheel);
		go_to_ball(left_wheel, right_wheel);
	}
	if (harvest_available() == -2) // obstacle on foward
	{
		std::cout << "the obstacle is on middle" << std::endl;
		left_wheel_msg.data = -DEG2RAD(100); //move back
		right_wheel_msg.data = -DEG2RAD(100);
		left_wheel.publish(left_wheel_msg);
		right_wheel.publish(right_wheel_msg);
		ros::Duration(2).sleep();

		left_wheel_msg.data = DEG2RAD(200); //rotate CW
		right_wheel_msg.data = -DEG2RAD(200);
		left_wheel.publish(left_wheel_msg);
		right_wheel.publish(right_wheel_msg);
		ros::Duration(2).sleep();

		left_wheel_msg.data = DEG2RAD(200); //move forward
		right_wheel_msg.data = DEG2RAD(200);
		left_wheel.publish(left_wheel_msg);
		right_wheel.publish(right_wheel_msg);
		ros::Duration(3).sleep();

		left_wheel_msg.data = -DEG2RAD(100); //rotate CCW
		right_wheel_msg.data = DEG2RAD(100);
		left_wheel.publish(left_wheel_msg);
		right_wheel.publish(right_wheel_msg);
		ros::Duration(2).sleep();

		align(left_wheel, right_wheel);
		go_to_ball(left_wheel, right_wheel);
	}
	if (harvest_available() == 0) //should turn left
	{
		std::cout << "the obstacle is on right" << std::endl;
		left_wheel_msg.data = -DEG2RAD(100); //rotate CW
		right_wheel_msg.data = DEG2RAD(100);
		left_wheel.publish(left_wheel_msg);
		right_wheel.publish(right_wheel_msg);
		ros::Duration(3).sleep();

		left_wheel_msg.data = DEG2RAD(200); //go forward
		right_wheel_msg.data = DEG2RAD(200);
		left_wheel.publish(left_wheel_msg);
		right_wheel.publish(right_wheel_msg);
		ros::Duration(3).sleep();
		align(left_wheel, right_wheel);
		go_to_ball(left_wheel, right_wheel);
	}
	ros::Duration(0.3).sleep();
}

void catch_ball(ros::Publisher left_wheel, ros::Publisher right_wheel, ros::Publisher gripper_joint, ros::Publisher el_gripper)
{
	std::cout << "catch the ball" << std::endl;
	close_gripper(gripper_joint);
	ros::Duration(4).sleep();
	if(lidar_distance[180]<0.4){
		move_back(left_wheel, right_wheel);
	}
	gripper_up(el_gripper);
	ros::Duration(7).sleep();
}

void find_goal(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	int count = 0;
	std::cout << "find goal function is called" << std::endl;

	while (ros::ok)
	{
		if (lidar_distance[180]<0.4 && count > 500) {
			std::cout << "too much rotate, find the goal" << std::endl;
			move_back(left_wheel, right_wheel);
		}
		ros::spinOnce();
		if (goal_angle > -0.1 && goal_angle < 0.1)
		{
			std::cout << goal_angle << std::endl;
			break;
		}
		if (goal_angle < 0)
		{ //rotate clockwise direction
			std::cout << "find goal : rotate CW" << std::endl;
			if (goal_angle > -5)
			{
				left_wheel_msg.data = DEG2RAD(100);
				right_wheel_msg.data = -DEG2RAD(100);
			}
			else
			{
				left_wheel_msg.data = DEG2RAD(300);
				right_wheel_msg.data = -DEG2RAD(300);
			}
		}
		else
		{ //rotate counter-clockwise direction s
			std::cout << "find goal : rotate CCW" << std::endl;
			if (goal_angle < 5)
			{
				left_wheel_msg.data = -DEG2RAD(100);
				right_wheel_msg.data = DEG2RAD(100);
			}
			else
			{
				left_wheel_msg.data = -DEG2RAD(300);
				right_wheel_msg.data = DEG2RAD(300);
			}
		}
		left_wheel.publish(left_wheel_msg);
		right_wheel.publish(right_wheel_msg);
	}
	stop(left_wheel, right_wheel);
}

void goto_goal(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	float min_angle = 999;
	for(int i=0 ; i< red_count ; i++){
		if(abs(ball_angle[i]) < min_angle){
			min_angle = abs(ball_angle[i]);
		}
	}
	ros::spinOnce();
	if (no_obstacle() == 1)
	{
		std::cout << "no obstacle " << ball_dist << std::endl;
		
		while (ros::ok())
		{
			ros::spinOnce();
			if (green_count == 0)
			{
				find_goal(left_wheel, right_wheel);
			}
			std::cout << "green_ball_dist : " << goal_distance << std::endl;
			left_wheel_msg.data = max_vel;
			right_wheel_msg.data = max_vel;
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);
			if(no_obstacle()!=1){
				break;
			}
			if (goal_distance < 0.6)
			{
				stop(left_wheel, right_wheel);
				find_goal(left_wheel, right_wheel);
				ros::Duration(0.5).sleep();
				break;
			}
		}
		while (ros::ok())
		{
			ros::spinOnce();
			if (no_obstacle() != 1)
				break;
			std::cout << "goal_dist(<0.6) : " << goal_distance << std::endl;
			left_wheel_msg.data = max_vel;
			right_wheel_msg.data = max_vel;
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);
			if (goal_distance < 0.4)
			{
				ros::Duration(1.1).sleep();
				stop(left_wheel, right_wheel);
				break;
			}
		}
	}
	if (no_obstacle() == -1)
	{ //should turn right
			std::cout << "the obstacle is on left" << std::endl;
			left_wheel_msg.data = DEG2RAD(100); //rotate CW
			right_wheel_msg.data = -DEG2RAD(100);
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);
			ros::Duration(3).sleep();

			left_wheel_msg.data = DEG2RAD(200); //go forward
			right_wheel_msg.data = DEG2RAD(200);
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);
			ros::Duration(6).sleep();
			find_goal(left_wheel, right_wheel);
			goto_goal(left_wheel, right_wheel);
		}
		if (no_obstacle() == -2) // obstacle on foward
		{
			std::cout << "the obstacle is on middle" << std::endl;
			left_wheel_msg.data = -DEG2RAD(100); //move back
			right_wheel_msg.data = -DEG2RAD(100);
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);
			ros::Duration(3).sleep();

			left_wheel_msg.data = DEG2RAD(200); //rotate CW
			right_wheel_msg.data = -DEG2RAD(200);
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);
			ros::Duration(3).sleep();

			left_wheel_msg.data = DEG2RAD(200); //move forward
			right_wheel_msg.data = DEG2RAD(200);
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);
			ros::Duration(3).sleep();

			left_wheel_msg.data = -DEG2RAD(100); //rotate CCW
			right_wheel_msg.data = DEG2RAD(100);
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);
			ros::Duration(3).sleep();

			find_goal(left_wheel, right_wheel);
			goto_goal(left_wheel, right_wheel);
		}
		if (no_obstacle() == 0) //should turn left
		{
			std::cout << "the obstacle is on right" << std::endl;
			left_wheel_msg.data = -DEG2RAD(100); //rotate CW
			right_wheel_msg.data = DEG2RAD(100);
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);
			ros::Duration(3).sleep();

			left_wheel_msg.data = DEG2RAD(200); //go forward
			right_wheel_msg.data = DEG2RAD(200);
			left_wheel.publish(left_wheel_msg);
			right_wheel.publish(right_wheel_msg);
			ros::Duration(6).sleep();
			find_goal(left_wheel, right_wheel);
			goto_goal(left_wheel, right_wheel);
		}
		ros::Duration(0.3).sleep();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "data_integation_tB_node");
	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
	ros::Subscriber time = n.subscribe<std_msgs::Float32>("/simTime", 10, time_Callback);

	ros::Publisher el_grip = n.advertise<std_msgs::Float64>("/elgrip_joint_pos", 10);
	ros::Publisher gripper = n.advertise<std_msgs::Float64>("/grip_joint_pos", 10);

	ros::Publisher legs = n.advertise<std_msgs::Float64>("/legs_joint_pos", 10);

	ros::Publisher pub_left_wheel = n.advertise<std_msgs::Float64>("/LWheel_joint", 10);
	ros::Publisher pub_right_wheel = n.advertise<std_msgs::Float64>("/RWheel_joint", 10);

	ros::Duration(2).sleep(); //this command is necessary for publishing

	ros::spinOnce();
	std::cout << lidar_size << std::endl;

	int i = 0;
	while (ros::ok())
	{
		std::cout << "Ball harvesting zone " << std::endl;

		stat_init(pub_left_wheel, pub_right_wheel, legs, gripper, el_grip);
		ros::Duration(12).sleep();
		ros::Duration(0.5).sleep();
		align(pub_left_wheel, pub_right_wheel);
		ros::Duration(0.5).sleep();
		ros::Duration(0.5).sleep();
		ros::spinOnce();
		std::cout << "is pole there?" << harvest_available() << std::endl;
		ros::Duration(5).sleep();
		go_to_ball(pub_left_wheel, pub_right_wheel);
		ros::Duration(0.5).sleep();
		catch_ball(pub_left_wheel, pub_right_wheel, gripper, el_grip);
		ros::Duration(3).sleep();
		ros::Duration(0.5).sleep();
		find_goal(pub_left_wheel, pub_right_wheel);
		ros::Duration(0.5).sleep();
		ros::Duration(0.5).sleep();
		goto_goal(pub_left_wheel, pub_right_wheel);
		ros::Duration(0.5).sleep();
		open_gripper(gripper);
		ros::Duration(0.5).sleep();
		move_back(pub_left_wheel, pub_right_wheel);
		i++;
	}
	return 0;
}

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
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "core_msgs/lane_position.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

#include "opencv2/opencv.hpp"


#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180)

//velocity
int max_vel=DEG2RAD(400);
int min_vel=DEG2RAD(100);
int min_ang_vel=DEG2RAD(80);
int max_ang_vel=DEG2RAD(400);

int is_line = 1;
int is_step = 0;
int phase=0;
float cen_x[10], cen_y[10];

float percent1; // percentage of the line
float percent2;
float percent3;

float diff1;
float diff2;
// w = 640, h = 480

std_msgs::Int32 msg_exit;

double simtime = 0;

void step_Callback(const std_msgs::Int32::ConstPtr& msg){
	is_step = msg->data;
}

void time_Callback(const std_msgs::Float32::ConstPtr& msg){
	simtime = msg->data;
	std::cout << "simTime is :    " << simtime << std::endl;
}


void line_Callback(const core_msgs::lane_position::ConstPtr& line_argument)
{
	percent1 = line_argument -> w1;
	percent2 = line_argument -> w2;
	percent3 = line_argument -> w3;
	diff1 = percent2 - percent1;
	diff2 = percent2 - percent3;
	is_line = line_argument -> size;
}



void stop(ros::Publisher left_wheel, ros::Publisher right_wheel){
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data = 0;
	right_wheel_msg.data = 0;
	left_wheel.publish(left_wheel_msg);  
	right_wheel.publish(right_wheel_msg);
	ros::Duration(0.5).sleep(); 
	std::cout << "stop process is done!!!" << std::endl;
}


int is_straight(){
	std::cout << percent2 << std::endl;
	if(!is_line) return 1;
	if(percent2 > 70){
		if(phase!=0) return 1;
		else{
			if(percent2 > 80) return 1;
		}
	}
	if(percent1 < percent3) return 0; //turn right
	else return -1; //turn left
}

void set_init(ros::Publisher left_wheel, ros::Publisher right_wheel){
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data=0;
	right_wheel_msg.data=0;
	left_wheel.publish(left_wheel_msg);
	right_wheel.publish(right_wheel_msg);
}

void front_lift(ros::Publisher front_leg){
	std_msgs::Float64 front_leg_msg;
	front_leg_msg.data=0;
	front_leg.publish(front_leg_msg);
}

void rear_lift(ros::Publisher rear_leg){
	std_msgs::Float64 rear_leg_msg;
	rear_leg_msg.data=0;
	rear_leg.publish(rear_leg_msg);
}

void go_straight(ros::Publisher left_wheel, ros::Publisher right_wheel){
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	std::cout << "go straight" << std::endl;
	left_wheel_msg.data = max_vel;
	right_wheel_msg.data = max_vel;
	left_wheel.publish(left_wheel_msg);  
	right_wheel.publish(right_wheel_msg); 
}


void climb_stair(ros::Publisher left_wheel, ros::Publisher right_wheel, ros::Publisher front_leg, ros::Publisher rear_leg, ros::Publisher legs){
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	std_msgs::Float64 leg_msg;
	//Before Climbing step
	left_wheel_msg.data = max_vel;
	right_wheel_msg.data = max_vel;
	left_wheel.publish(left_wheel_msg);  
	right_wheel.publish(right_wheel_msg); 
	ros::spinOnce();
	ros::Duration(3).sleep();

	//After the front wheel climb the step
	stop(left_wheel, right_wheel);
	std::cout << "lift the front leg" << std::endl;
	front_lift(front_leg);
	ros::spinOnce();
	ros::Duration(15).sleep();
	std::cout << "change the velocity" << std::endl;
	left_wheel_msg.data = max_vel;
	right_wheel_msg.data = max_vel;
	left_wheel.publish(left_wheel_msg);  
	right_wheel.publish(right_wheel_msg);
	ros::Duration(7).sleep();
	std::cout <<"lift the rear leg" << std::endl;
	float start = simtime;
	rear_lift(rear_leg);
	ros::Duration(17).sleep();
	stop(left_wheel, right_wheel);

	ros::Duration(1).sleep();
	leg_msg.data = -0.15;
	legs.publish(leg_msg);
	ros::Duration(17).sleep();
}

void turn_left(ros::Publisher left_wheel, ros::Publisher right_wheel){
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	std::cout << "turn left" << std::endl;
	left_wheel_msg.data = min_ang_vel;
	right_wheel_msg.data = max_ang_vel;
    left_wheel.publish(left_wheel_msg);  
	right_wheel.publish(right_wheel_msg); 

}
void turn_right(ros::Publisher left_wheel, ros::Publisher right_wheel){
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	std::cout << "turn right" << std::endl;
	left_wheel_msg.data = max_ang_vel;
	right_wheel_msg.data = min_ang_vel;	
    left_wheel.publish(left_wheel_msg);  
	right_wheel.publish(right_wheel_msg); 
}


void drive(ros::Publisher left_wheel, ros::Publisher right_wheel){
	while(ros::ok()){
		std::cout << "simulation time is : " << simtime << std::endl;
		ros::spinOnce();
		if(simtime>25 && is_step){ros::Duration(1).sleep(); break;}
		if(is_straight()==1){
			go_straight(left_wheel, right_wheel);
		}
		else if(is_straight()==0){
			turn_left(left_wheel, right_wheel);
		}
		else{
			turn_right(left_wheel, right_wheel);
		}
	}
}


void final(ros::Publisher left_wheel, ros::Publisher right_wheel){
	while(ros::ok()){
		std::cout << "simulation time is : " << simtime << std::endl;
		ros::spinOnce();
		if((percent1 || percent2 || percent3 )==0)  break;
		if(simtime>70) break;

		if(is_straight()==1){
			go_straight(left_wheel, right_wheel);
		}
		else if(is_straight()==0){
			turn_left(left_wheel, right_wheel);
		}
		else{
			turn_right(left_wheel, right_wheel);
		}
	}
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "line_trace_node");
    ros::NodeHandle n;
	msg_exit.data = 0;
	
	ros::Subscriber sub1 = n.subscribe<core_msgs::lane_position>("/line_argument", 1000, line_Callback);
	ros::Subscriber time = n.subscribe<std_msgs::Float32>("/simTime", 10, time_Callback);
	ros::Subscriber is_step = n.subscribe<std_msgs::Int32>("/step_detect", 1, step_Callback);

	ros::Publisher pub_exit_call = n.advertise<std_msgs::Int32>("/exit_call", 10);
	ros::Publisher front_leg = n.advertise<std_msgs::Float64>("/legF_joint_pos", 10);
	ros::Publisher rear_leg = n.advertise<std_msgs::Float64>("/legR_joint_pos", 10);
	ros::Publisher legs = n.advertise<std_msgs::Float64>("/legs_joint_pos", 10);

	ros::Publisher pub_left_wheel= n.advertise<std_msgs::Float64>("/LWheel_joint", 10);
	ros::Publisher pub_right_wheel= n.advertise<std_msgs::Float64>("/RWheel_joint", 10);

    ros::Duration(3).sleep();
	//climb_stair(pub_left_wheel, pub_right_wheel, front_leg, rear_leg);
	
	ros::spinOnce();

	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data = 3;
	right_wheel_msg.data = 3;
	pub_left_wheel.publish(left_wheel_msg);
	pub_right_wheel.publish(right_wheel_msg);
	ros::Duration(3).sleep();

	climb_stair(pub_left_wheel, pub_right_wheel, front_leg, rear_leg, legs);
    return 0;
}

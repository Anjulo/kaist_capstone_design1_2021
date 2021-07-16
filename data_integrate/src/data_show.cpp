#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "core_msgs/ball_position.h"

#include "opencv2/opencv.hpp"

float MAP_CX = 200.5;
float MAP_CY = 200.5;
float MAP_RESOL = 0.015;             // Map resoultion [cm]
int MAP_WIDTH = 400;
int MAP_HEIGHT = 400;
int MAP_CENTER = 50;
int OBSTACLE_PADDING = 2;           // Obstacle Size
int OBSTACLE_CONNECT_MAX = 15;      // Range to connect obstacles

int init_ball;
int init_lidar;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];

int ball_number;
float ball_X[20];
float ball_Y[20];

float ball_distance[20];
float ball_angle[20];

float goal_distance;
float goal_angle;
float min_ball_angle;
float min_distance=9999999;

boost::mutex map_mutex;


#define RAD2DEG(x) ((x)*180./M_PI)

bool check_point_range(int cx, int cy)
{
    return (cx<MAP_WIDTH-1)&&(cx>0)&&(cy<MAP_HEIGHT-1)&&(cy>0);
}



void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
    map_mutex.lock();
    int red_count = position->size_red;
	int green_count = position -> size_green;
    for(int i = 0; i < red_count; i++)
    {
		ball_distance[i]=position->distance_red[i];
		ball_angle[i]=position->theta_red[i];

		std::cout << "red_degree : "<< position->theta_red[i];
		std::cout << "		red_distance : "<< position->distance_red[i] << std::endl;
		if(position->distance_red[i]<=min_distance){
			min_distance = position->distance_red[i];
			min_ball_angle = position->theta_red[i];
		}

    }

	for(int i=0; i<green_count ; i++){
		std::cout << "degree : "<< position->theta_green[i];
		std::cout << "		distance : "<< position->distance_green[i] << std::endl;
		goal_distance = position->distance_green[i];
		goal_angle = position->theta_green[i];
	}
}
void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    map_mutex.lock();
    int count = scan->angle_max / scan->angle_increment;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
        lidar_distance[i]=scan->ranges[i];
        // std::cout << "degree : "<< lidar_degree[i];
        // std::cout << "   distance : "<< lidar_distance[i]<<std::endl;
    }
    map_mutex.unlock();

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_show_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);

    while(ros::ok){
        cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);
        // Drawing Lidar data
        float obstacle_x, obstacle_y;
        int cx, cy;
        int cx1, cx2, cy1, cy2;
        for(int i = 0; i < lidar_size; i++)
        {
            obstacle_x = lidar_distance[i]*cos(lidar_degree[i]);
            obstacle_y = lidar_distance[i]*sin(lidar_degree[i]);
            cx = MAP_WIDTH/2 + (int)(obstacle_y/MAP_RESOL);
            cy = MAP_HEIGHT/2 + (int)(obstacle_x/MAP_RESOL);
            cx1 = cx-OBSTACLE_PADDING;
            cy1 = cy-OBSTACLE_PADDING;
            cx2 = cx+OBSTACLE_PADDING;
            cy2 = cy+OBSTACLE_PADDING;

            if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
            {
                cv::line(map, cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),cv::Point(cx, cy),cv::Scalar(63,63,0), 1);
                cv::rectangle(map,cv::Point(cx1, cy1),cv::Point(cx2, cy2), cv::Scalar(255,255,0), -1);
            }
        }
        // Drawing ball
        for(int i = 0; i < ball_number; i++)
        {
            cx =(int)(ball_X[i]/4);
            cy =(int)(ball_Y[i]/4);
            cx1 = cx-OBSTACLE_PADDING*2;
            cy1 = cy-OBSTACLE_PADDING*2;
            cx2 = cx+OBSTACLE_PADDING*2;
            cy2 = cy+OBSTACLE_PADDING*2;

            if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
            {
                cv::rectangle(map,cv::Point(cx1, cy1),cv::Point(cx2, cy2), cv::Scalar(0,0,255), -1);
            }
        }
        // Drawing ROBOT
        cv::circle(map,cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),3,cv::Scalar(255,0,0),-1);

        cv::imshow("Frame",map);
        cv::waitKey(50);

        if(cv::waitKey(50)==113){  //wait for a key command. if 'q' is pressed, then program will be terminated.
            return 0;
        }
        ros::spinOnce();
    }



    return 0;
}

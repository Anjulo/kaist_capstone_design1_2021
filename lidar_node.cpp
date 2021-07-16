#include <ros/ros.h>
#include "string.h"
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>

#include <vector>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <geometry_msgs/Point.h>

using namespace cv;

std_msgs::Float32MultiArray msg_x;
std_msgs::Float32MultiArray msg_y;
std_msgs::Float32MultiArray msg_wall;
//std_msgs::Float32MultiArray msg_ran;
//std_msgs::Float32MultiArray msg_ang;
int cb_count = 0;
int len;
std::vector<int> pole_wall1;
std::vector<int> pole_wall2;


void lidar_cb(sensor_msgs::LaserScan msg){
	cb_count += 1;
	
	pole_wall1 = {};
	pole_wall2 = {};
	
	
	msg_x.data = {};
	msg_y.data = {};
	//msg_ran.data = {};
	//msg_ang.data = {};
    // angle in radian
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    std::vector<float> range = msg.ranges;

    len = range.size();
    float angle[len];
    float angle_temp;

	
	
	std::vector<std::vector<int>> pole_wall;

    Mat plot_result;
    Ptr<plot::Plot2d> plot_xy;
    Mat x(Size(1, len), CV_64F, Scalar(0));
    Mat y(Size(1, len), CV_64F, Scalar(0));

    for(int i = 0; i < len-1; i++){
        angle_temp = angle_min + i*angle_increment;
        angle[i] = angle_temp;
        if (std::isinf(range[i])==false && range[i]*sin(angle_temp)!= -0.000000 ){
            x.at<double>(i) = range[i]*cos(angle_temp);
            y.at<double>(i) = -range[i]*sin(angle_temp);
        }
    }

    plot_xy = plot::Plot2d::create(x,y);

    plot_xy->setMaxX(5);
    plot_xy->setMinX(-5);
    plot_xy->setMaxY(5);
    plot_xy->setMinY(-5);
    plot_xy->setPlotSize(720,720);
    plot_xy->setNeedPlotLine(false);
    plot_xy->render(plot_result);

    imshow("plot", plot_result);
    waitKey(1);

// find range of indexes of poles
// pole_wall1 : storing start index of pole, pole_wall2 : storing end index of pole

	for (int i = 0; i < len; i++)
  	{
    	if (i != len - 1)
    	{
			if ((range[i + 1] - range[i]) <= -0.7)
				pole_wall1.push_back(i + 1);
			else if ((range[i + 1] - range[i]) >= 0.7)
        		pole_wall2.push_back(i);
			
    	}
    	
    	else	//i = len - 1
    	{
    		if ((range[len - 1] - range[0]) <= -0.7)
		    	pole_wall1.push_back(len - 1);
		  	else if ((range[len - 1] - range[0]) >= 0.7)
		    	pole_wall2.push_back(len);
		}
		
    }
	

	int min_index = 0;
	int min_dif = 360;
	int dif = 360;
	int len_pole1 = pole_wall1.size();
	int len_pole2 = pole_wall2.size();
	
	if (len_pole1 > len_pole2){
		ROS_INFO("lidar error 1");
		msg_x.data.push_back(0);
		msg_y.data.push_back(0);
		return;
	}
	
	if (len_pole1 == 0 | len_pole2 == 0){
		ROS_INFO("lidar error 2");
		msg_x.data.push_back(0);
		msg_y.data.push_back(0);
		return;
	}
	
	int flag = 0;
	int length = 0;
	int over360 = 0;
	for (int i = 0; i < len_pole1; i++){
		for (int j = 0; j < len_pole2; j++){
			if (pole_wall2[j] > pole_wall1[i]){
				dif = -pole_wall1[i] + pole_wall2[j];
				if (dif < min_dif){
					min_dif = dif;
					min_index = j;
					over360 = 0;
				}
			}
			else {
				dif = 360 - pole_wall1[i] + pole_wall2[j];
				if (dif < min_dif){
					min_dif = dif;
					min_index = j;
					over360 = 1;
				}
			}
		}

		if (min_dif < 20){
			std::vector<int> point_xy;
			point_xy.push_back(pole_wall1[i]);
			if (over360) point_xy.push_back(360 + pole_wall2[min_index]);
			else point_xy.push_back(pole_wall2[min_index]);
			pole_wall.push_back(point_xy);
			length += 1;
			min_dif = 360;
			over360 = 0;
		}
	}
	
	// 중복 에러 체크
	int error_check = 361;
	for (int i = 0; i < length; i++){
		for (int j = i + 1; j < length; j++){
			if (pole_wall[i][1] == pole_wall[j][1]){
				
				msg_x.data.push_back(0);
				msg_y.data.push_back(0);
				ROS_INFO("lidar error 3, %d", cb_count);
				return;
				
			}
		}
		//ROS_INFO("%d, %d", pole_wall[i][0], pole_wall[i][1]);
	}
	//ROS_INFO("%d %d %d %d", len_pole1, len_pole2, length, cb_count);
	
	/*
	if (pole_wall1[0] < pole_wall2[0]){
		for (int i = 0; i < len_pole2; i++){
			pole_wall2_mod.push_back(pole_wall2[i]);
		}
	} else{
		for (int i = 0; i < len_pole2; i++){
			if (i = len_pole2 - 1){
				pole_wall2_mod.push_back(pole_wall2[0]);
			}
			else {
				pole_wall2_mod.push_back(pole_wall2[i + 1]);
			}
		}
	}
	*/
	
	msg_x = {};
	msg_y = {};
	float avg_angle;
	int mid_index;
	for (int i = 0; i < length; i++){
		mid_index = (pole_wall[i][0] + pole_wall[i][1]) / 2;
		avg_angle = angle_min + angle_increment * mid_index;
		
		msg_x.data.push_back(range[mid_index] * cos(avg_angle));
        msg_y.data.push_back(-range[mid_index] * sin(avg_angle));
	}


	//wallllllllllll
	
	std::vector<int> poleindices = {};
	std::vector<int> wallpoints = {};
	
	std::vector<float> wall_directions = {};

	for (int i = 0; i < length; i++){
	
		int pole1 = pole_wall[i][0];
		int pole2 = pole_wall[i][1];
		int over_cap = 0;
		if (pole2 >= 360){
			over_cap = 1;
		}
		if (over_cap){
			for (int j = 0; j <= pole2-360; j++){
				poleindices.push_back(j);
			}
			for (int j = pole1; j <= 360; j++){
				poleindices.push_back(j);
			}
			continue;
		}
		
		if (pole1 < pole2){
			for (int j = pole1; j <= pole2; j++){
				poleindices.push_back(j);
			}
		} else{
			for (int j = pole2; j <= pole1; j++){
				poleindices.push_back(j);
			}
		}
	}
	//ROS_INFO("%ld", poleindices.size());
	
	
	for (int i = 0; i < len; i++)
  	{
  		int comp;
    	for (int j = 0; j < poleindices.size(); j++){
    		if (i == poleindices[j]) comp = 1;
    	}
    	if (comp == 1) {comp = 0; continue;}
    	wallpoints.push_back(i);
    }
    
    /*
    int comp;
	float ang;

	float F=1000, FR=1000, R=1000, RB=1000, B=1000, BL=1000, L=1000, LF=1000;
	for (int i = 0; i < len; i++){
		
    	if (range[i] > 1){
			continue;
		}
		for (int j = 0; j < poleindices.size(); j++){
    		if (i == poleindices[j]) comp = 1;
    	}
    	if (comp == 1) {comp = 0; continue;}
    	//wallpoints.push_back(i);
		
		ang = angle_min + i*angle_increment;
		
		if (i >= 0 %% i < 23) 
	}
	
	
	msg_wall.data = {};
		
	msg_wall.data.push_back(F); msg_wall.data.push_back(FR); msg_wall.data.push_back(R); msg_wall.data.push_back(RB);
	msg_wall.data.push_back(B); msg_wall.data.push_back(BL); msg_wall.data.push_back(L); msg_wall.data.push_back(LF);
	ROS_INFO("%d, %d, %d, %d %f", F, FR, R, RB, minf);
	*/

	/*
	float prev = wallpoints[wallpoints.size()-1];
	float now, after;
	std::vector<int> wall_closest_points;
	for (int i = 0; i < wallpoints.size(); i++){
		now = wallpoints[i];
		if (i != wallpoints.size())
			after = wallpoints[i+1];
		else after = wallpoints[0];
		
		if (now < prev && now < after){
			wall_closest_points.push_back(i);
			msg_ran.data.push_back(range[i]);
			msg_ang.data.push_back(angle_min + i*angle_increment);
		}
		prev = now;
	}
	*/
	float F=1000, FR=1000, R=1000, RB=1000, B=1000, BL=1000, L=1000, LF=1000;
	std::vector<int> dir_index = {180, 135, 90, 45, 0, 315, 270, 225};
	float ang;
	
	for (int i = 0; i < dir_index.size(); i++){
		int comp = 0;
		for (int j = 0; j < poleindices.size(); j++){
			if (poleindices[j] == dir_index[i]) comp = 1;
		}
		if (comp == 1) continue;
		ang = angle_min + dir_index[i]*angle_increment;
		if (range[dir_index[i]] == 0){
			ROS_INFO("range error");
			return;
		}
		wall_directions.push_back(range[dir_index[i]]);
		
	}
	ROS_INFO("%f, %f, %f, %f", wall_directions[0], wall_directions[1], wall_directions[2], wall_directions[3]);
	
    plot_result.release();
    x.release();
    y.release();
    range.clear();
    
	
	
	
	
}





int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_lidar = nh.subscribe("/scan", 1, lidar_cb);
    ros::Publisher pub_lidar_x = nh.advertise<std_msgs::Float32MultiArray>("/lidar_x",1);
    ros::Publisher pub_lidar_y = nh.advertise<std_msgs::Float32MultiArray>("/lidar_y",1);
    ros::Publisher pub_wall = nh.advertise<std_msgs::Float32MultiArray>("/wall",1);
    //ros::Publisher pub_wall_ran = nh.advertise<std_msgs::Float32MultiArray>("/wall_range",1);
	//ros::Publisher pub_wall_ang = nh.advertise<std_msgs::Float32MultiArray>("/wall_angle",1);

    ros::Rate loop_rate(5);

    while(ros::ok()){
        ros::spinOnce();
      
        pub_lidar_x.publish(msg_x);
        pub_lidar_y.publish(msg_y);
        //pub_wall_ran.publish(msg_ran);
        //pub_wall_ang.publish(msg_ang);
        pub_wall.publish(msg_wall);

        loop_rate.sleep();
    }
    return 0;
}

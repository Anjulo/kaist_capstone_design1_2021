#include <ros/ros.h>
#include "string.h"
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>

#include <std_msgs/Int32.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

Mat buffer;
Mat buff_blurred;

std_msgs::Int32 msg;

void morphOps(Mat &thresh);

//void pointcloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg){
int is_step = 0;
int is_lane = 0;

void depthImage_cb(const sensor_msgs::ImageConstPtr &msg){

	try{

		cv_bridge::CvImageConstPtr depth_img_cv;
		Mat depth_mat;	// size : 
		// Get the ROS image to openCV
		depth_img_cv = cv_bridge::toCvShare (msg, sensor_msgs::image_encodings::TYPE_16UC1);
		// Convert the uints to floats
		depth_img_cv->image.convertTo(depth_mat, CV_32F, 0.001); // pixel : 0 ~ 1

		float mean_down1 = 0;
		float mean_down2 = 0;
		float mean_down3 = 0;
		float mean_down4 = 0;
		float mean_down5 = 0;
		float mean_up = 0;

		int h = depth_mat.rows;
		int w = depth_mat.cols;
		//ROS_INFO("width : %d, height : %d, %f, %f", h, w, depth_mat.at<float>(200, 200));

		//Mat test_mat(480, 640, CV_32F);
		for (int i = 0; i < 480; i++){
			for (int j = 0; j < 640; j++){
				//test_mat.at<float>(i, j) = 0.2;
				
				//if (depth_mat.at<float>(i, j) > 1) depth_mat.at<float>(i, j) = 1;
				
				
				if (( 480 * 16/20 > i && i >= 480 * 3/4) && (640 * 2/5 <= j && j <= 640 * 3/5))
					mean_down1 += depth_mat.at<float>(i, j);
				else if (( 480 * 17/20 > i && i >= 480 * 16/20) && (640 * 2/5 <= j && j <= 640 * 3/5))
					mean_down2 += depth_mat.at<float>(i, j);
				else if (( 480 * 18/20 > i && i >= 480 * 17/20) && (640 * 2/5 <= j && j <= 640 * 3/5))
					mean_down3 += depth_mat.at<float>(i, j);
				else if (( 480 * 19/20 > i && i >= 480 * 18/20) && (640 * 2/5 <= j && j <= 640 * 3/5))
					mean_down4 += depth_mat.at<float>(i, j);
				else if (( 480 * 20/20 > i && i >= 480 * 19/20) && (640 * 2/5 <= j && j <= 640 * 3/5))
					mean_down5 += depth_mat.at<float>(i, j);
				else if (480 * 2/5 >= i && (640 * 1/5 <= j && j <= 640 * 4/5))
					mean_up += depth_mat.at<float>(i, j);			
			}
		}
		mean_down1 /= (640 * 1/5)*480 *1/20;
		mean_down2 /= (640 * 1/5)*480 *1/20;
		mean_down3 /= (640 * 1/5)*480 *1/20;
		mean_down4 /= (640 * 1/5)*480 *1/20;
		mean_down5 /= (640 * 1/5)*480 *1/20;
		mean_up /= (640 * 3/5)*480 *2/5;
		
		ROS_INFO("1 : %f, 2 : %f, 3 : %f, 4 : %f, 5 : %f, up : %f", mean_down1, mean_down2, mean_down3, mean_down4, mean_down5, mean_up);
		//imshow("view2", test_mat);
		
		
		imshow("view", depth_mat);
		cv::waitKey(1);
		
		is_step = 1;
		if (!(mean_up > 1.4)) is_step = 0;
		if (!(mean_down1 > 0.3 && mean_down1 < 0.34)) is_step = 0;
		if (!(mean_down2 > 0.27 && mean_down2 < 0.31)) is_step = 0;
		if (!(mean_down3 > 0.25 && mean_down3 < 0.29)) is_step = 0;
		if (!(mean_down4 > 0.23 && mean_down4 < 0.27)) is_step = 0;
		if (!(mean_down5 > 0.21 && mean_down5 < 0.25)) is_step = 0;

	

	}

	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}

}

void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(3,3));

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
  dilate(thresh,thresh,dilateElement);
  dilate(thresh,thresh,dilateElement);
  //dilate(thresh,thresh,dilateElement);

	//erode(thresh,thresh,erodeElement);
	//erode(thresh,thresh,erodeElement);

}

void lane_detect(){

  is_lane = 0;
  //===================Variable Declaration=============================
  Mat edges_black;  //assign a memory to save the edge images
  Mat hsv, hsv_black, roi, roi_1, roi_2, roi_3, roi_mu;
  int w1,w2,w3, wt;
  int h, w;
  vector<Point> locations, cont_1, cont_2, cont_3;
  int num_str;
  //=======================================================================
  //=================Color thresholding in HSV space============================
  GaussianBlur(buffer, buff_blurred, Size(3, 3), 0, 0);
  cvtColor(buff_blurred, hsv, COLOR_BGR2HSV);
  
  inRange(hsv, Scalar(0,0,0), Scalar(180,255,30), hsv_black);
  //=======================================================================

  //=========== Morpholigical image proccessing to remove any noise =======
  //
  morphOps(hsv_black); 
  GaussianBlur(hsv_black, hsv_black, Size(11, 11), 2, 2);
  //=======================================================================

  //===================Lane position======================================
  cv::Size sz = hsv_black.size();
  h = sz.height; w=sz.width;
  //bitwise_not(hsv_black,hsv_black);
  roi_1 = hsv_black(Rect(0,h/3,w/3,2*h/3));
  roi_2 = hsv_black(Rect(w/3,h/3,w/3,2*h/3));
  roi_3 = hsv_black(Rect(2*w/3,h/3,w/3,2*h/3));

  
  w1 = countNonZero(roi_1);
  w2 = countNonZero(roi_2);
  w3 = countNonZero(roi_3);
  wt = w1+w2+w3;
  
  //ROS_INFO("%d", wt);
  if (wt > 7000) is_lane = 1;
  
  


  //cv::imshow("view1", buffer);  //show the image with a window
  //cv::imshow("lane", hsv_black);
  
  //cv::imshow("edges_black", edges_black);
  ///cv::imshow("roi", roi);
  //cv::imshow("roi_1", roi_1); cv::imshow("roi_2", roi_2); cv::imshow("roi_3", roi_3);

  //cv::waitKey(1);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


  try
  {
    buffer = cv_bridge::toCvShare(msg, "bgr8")->image;  //transfer the image data into buffer
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  lane_detect(); //proceed lane detection
}








int main(int argc, char **argv){

    ros::init(argc, argv, "depth_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
  	image_transport::Subscriber sub_depth = it.subscribe("/kinect_depth", 1, depthImage_cb);
	image_transport::Subscriber sub = it.subscribe("/kinect_rgb", 1, imageCallback); //create subscriber
	
	ros::Publisher pub = nh.advertise<std_msgs::Int32>("/step_detect", 1);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        msg.data = is_step && is_lane;
        pub.publish(msg);
        ROS_INFO("%d", msg.data);
    }
    return 0;
}

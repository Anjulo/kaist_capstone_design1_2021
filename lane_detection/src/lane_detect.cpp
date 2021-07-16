#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/lane_position.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <cmath>

using namespace cv;
using namespace std;

float treshold = 100;

Mat buffer;
Mat buff_blurred;
ros::Publisher pub;
ros::Publisher pub_markers;

void morphOps(Mat &thresh);

//===============================
Size sz;
//===============================

void lane_detect(){

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
  
  inRange(hsv, Scalar(0,0,0), Scalar(180,255,100), hsv_black);
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

  //Intialize ROS message
  core_msgs::lane_position msg;  //create a message for lane positions

  //findNonZero(hsv_black, locations); // output, locations of lane pixels   
  w1 = countNonZero(roi_1);
  w2 = countNonZero(roi_2);
  w3 = countNonZero(roi_3);
  wt = w1+w2+w3;
  if(wt>=1){
    w1=(w1*100/wt);
    w2=(w2*100/wt);
    w3=(w3*100/wt);
  }
  //cout<<"Lane detected"<<endl;
  cout<<"w1="<<w1<<"% w2="<<w2<<"% w3="<<w3<<"%"<<endl;
    
  // publish the direction data to ros msg
  msg.w1=w1; msg.w2=w2; msg.w3=w3;

  //===================== moment detection =======
  num_str = 10;
  vector<Moments> mu(num_str);
  vector<Point2f> mc(num_str);

  msg.size = num_str;
  msg.com_x.resize(num_str);  //input the x position of the lane centroid to the message
  msg.com_y.resize(num_str);
  
  for(int i=0; i<num_str;i++){
    roi_mu = hsv_black(Rect(0,(10+2*i)*h/30,w,2*h/30));
    //roi_mu = contours_black(Rect(0,(2*i)*h/15,w,2*h/15));
    //cv::imshow("roi", roi);
    mu[i] = moments( roi_mu, false );                                               // Get the moments of image
    if(mu[i].m00 > 0){
      //cout<<i<<"- 1: " <<mu[i].m10/mu[i].m00<<" 2: " <<(mu[i].m01/mu[i].m00)+(3+2*i)*h/15<<endl;
      mc[i] = Point2f( mu[i].m10/mu[i].m00 , (mu[i].m01/mu[i].m00)+(10+2*i)*h/30 ); // Get the mass centers
      circle(hsv_black, mc[i],1, Scalar(0, 0, 0), 2);
      msg.com_x[i] = mu[i].m10/mu[i].m00; 
      msg.com_y[i] = (mu[i].m01/mu[i].m00)+(10+2*i)*h/30; 
    }
    else{
      mc[i] = Point2f( 0 , 0 ); 
      msg.com_x[i] = -1000; // invalid reasult to show there is no centroid
      msg.com_y[i] = -1000; 
    }

    //if(mu[i].m00 > 0)
    //  cout<<i<<"- 1: " <<mu[i].m10/mu[i].m00<<" 2: " <<(mu[i].m01/mu[i].m00)+(5+2*i)*h/30<<endl;
    //else 
    //  cout<<i<<"- 1: " <<0<<" 2: " <<0<<endl;
  }


  cv::imshow("view", buffer);  //show the image with a window
  cv::imshow("lane", hsv_black);
  //cv::imshow("edges_black", edges_black);
  ///cv::imshow("roi", roi);
  //cv::imshow("roi_1", roi_1); cv::imshow("roi_2", roi_2); cv::imshow("roi_3", roi_3);

  cv::waitKey(1);
  pub.publish(msg);  //publish a message
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_detect_node"); //init ros node
  ros::NodeHandle nh; //create node handler
  image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
  image_transport::Subscriber sub = it.subscribe("/kinect_rgb2", 1, imageCallback); //create subscriber

  pub = nh.advertise<core_msgs::lane_position>("/line_argument", 100); //setting publisher
  //pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);

  ros::spin(); //spin.
  return 0;
}






/// function definitions
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


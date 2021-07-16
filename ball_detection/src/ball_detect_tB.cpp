#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
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

vector<float> getBallPos();

void morphOps(Mat &thresh);

vector<float> getBallPos(float x, float y,float r);

//===============================
// Camera intrinsic parameters
float f = 589.37, w = 640, h = 480; // al in pixels
float cx = w/2, cy = h/2;  //(cx,cy) --> principal point
// prior map information
float R = 0.15/2;  // real ball radius
float L = 5, W=3; //length and width of the ball collection area
float max_dis = hypot(L,W); // the maximum distance the ball can be from the robot
float min_dis = 0.2;        // from observation
float minRadius = R*(f/max_dis);
float maxRadius = R*(f/min_dis);
//===============================

void ball_detect(){

  //===================Variable Declaration=============================
  Mat edges_red, edges_green;   //assign a memory to save the edge images
  Mat hsv, hsv_red_1, hsv_red_2, hsv_red, hsv_green;
  //=======================================================================
  //cv::imshow("view1", buffer);
  //=================Color thresholding in HSV space============================
  GaussianBlur(buffer, buff_blurred, Size(3, 3), 0, 0);
  cvtColor(buff_blurred, hsv, COLOR_BGR2HSV);
  
  //Threshold the color values
  // check this website for 
  //https://stackoverflow.com/questions/36817133/identifying-the-range-of-a-color-in-hsv-using-opencv
  //https://www.tydac.ch/color/
  inRange(hsv, Scalar(159,50,70), Scalar(180,255,255), hsv_red_1);
  inRange(hsv, Scalar(0,50,70), Scalar(9,255,255), hsv_red_2);
  addWeighted(hsv_red_1, 1.0, hsv_red_2, 1.0, 0.0, hsv_red);
  //inRange(hsv, Scalar(36,50,70), Scalar(89,255,255), hsv_green);
  inRange(hsv, Scalar(25, 50, 70), Scalar(35, 255, 255), hsv_green);
  //=======================================================================


  //=========== Morpholigical image proccessing to remove any noise =======
  //
  morphOps(hsv_red); 
  GaussianBlur(hsv_red, hsv_red, Size(9, 9), 2, 2);
  //
  morphOps(hsv_green);
  GaussianBlur(hsv_green, hsv_green, Size(9, 9), 2, 2);
  //=======================================================================
  
  // ================Circle detection =====================================
  Canny(hsv_red, edges_red, 100, 200, 3);
  Canny(hsv_green, edges_green, 100, 200, 3);

  // using findCountours()
  // https://github.com/akaifi/MultiObjectTrackingBasedOnColor
  // https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/bounding_rects_circles/bounding_rects_circles.html
  vector< vector<Point> > contours_red, contours_green;
	vector<Vec4i> hierarchy_red,hierarchy_green;
  findContours(edges_red, contours_red, hierarchy_red, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
  findContours(edges_green, contours_green, hierarchy_green, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

  Mat wall = Mat::zeros(buffer.size(),CV_8UC1);  //to draw contours on

  //Intialize ROS message
  core_msgs::ball_position msg;  //create a message for ball positions

  // Find contours
  Vec2f params; //assign a memory to save the information of centers
  vector<vector<Point> > contours_red_poly(contours_red.size() ), contours_green_poly(contours_green.size() );
  //vector<Rect> boundRect( contours_red.size() );
  vector<Point2f> centers_red( contours_red.size() ), centers_green( contours_green.size() );
  vector<float> radii_red( contours_red.size() ), radii_green( contours_green.size() );

  int num = 0; // number of balls detected
  float x,y,r;
  string info;
  vector<float> ballPos;
  float dis, theta; //(dis,theta) --> ball position in polar coordiate
  for(int k=0;k<contours_red.size();k=k+2){

    //============ for findContours()
    // Approximate contours to polygons and circles
    //approxPolyDP( contours_red[k], contours_red_poly[k], 1, true ); 
    minEnclosingCircle( contours_red[k], centers_red[k], radii_red[k] );


    params = centers_red[k];  //the information of k-th circle
    x=cvRound(params[0]);  //x position of k-th circle
    y=cvRound(params[1]);  //y position
    r=cvRound(radii_red[k]); //radius
    //===========
    if(minRadius<r and r<maxRadius){  // ball detected

      // draw circle
      Point center(x,y); //declare a Point 
      circle(buffer,center, 1, Scalar(0,0,0), 1, LINE_AA); // circle center
      circle(buffer,center,r,Scalar(255,0,0),2); // draw a circle on   
                                                
      drawContours(wall, contours_red, k, Scalar(255,0,0), 2);
      

      // Get the ball position in olar coordinates
      ballPos = getBallPos(x,y,r);
      dis = ballPos[0]; theta = ballPos[1];

      msg.size_red +=1;
      msg.distance_red.push_back(dis);  //input the x position of the ball to the message
      msg.theta_red.push_back(theta);

      //================ Put Text Information======================
      dis = truncf(dis * 10) / 10;
      theta = truncf(theta * 57.3 * 10) / 10;
      info = "  (" + to_string(dis) + ", " + to_string(theta) + ")";
      putText(buffer, info, center, cv::FONT_HERSHEY_DUPLEX, 0.3, Scalar(0,0,0), 0.5);

    }
  }
  cout<<"Number of circles="<<msg.size_red<<endl;  //print the number of circles detected

  //==========================================

  for(int k=0;k<contours_green.size();k=k+2){

    //============ for findContours()
    // Approximate contours to polygons + get bounding rects and circles
    //approxPolyDP( contours_green[k], contours_green_poly[k], 1, true );
    //boundRect[k] = boundingRect( contours_red_poly[k] );
    minEnclosingCircle( contours_green[k], centers_green[k], radii_green[k] );


    params = centers_green[k];  //the information of k-th circle
    x=cvRound(params[0]);  //x position of k-th circle
    y=cvRound(params[1]);  //y position
    r=cvRound(radii_green[k]); //radius
    //===========
    if(minRadius<r and r<maxRadius){  // ball detected

      // 원 출력을 위한 원 중심 생성 ==> Create a Circle Center for Circle Output
      Point center(x,y); //declare a Point 
      circle(buffer,center, 1, Scalar(0,0,0), 1, LINE_AA); // circle center
      circle(buffer,center,r,Scalar(0,255,255),2); // draw a circle on 'buffer' based on the information given,   
                                                  // r = radius, Scalar(0,0,255) means color, 10 means lineWidth
      
      //drawContours( buffer, contours_red_poly, k, Scalar(0,0,0), 1, 8, vector<Vec4i>(), 0, Point() );
      //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );

      // TO DO
      //convert the position of the ball in camera coordinate to the position in base coordinate. 
      // It is related to the calibration process. You shoould modify this.
      //===============================

      ballPos = getBallPos(x,y,r);
      dis = ballPos[0]; theta = ballPos[1];
      msg.size_green +=1;
      msg.distance_green.push_back(dis);  //input the x position of the ball to the message
      msg.theta_green.push_back(theta);

      //================ Put Text Information======================
      dis = truncf(dis * 10) / 10;
      theta = truncf(theta * 57.3 * 10) / 10;
      info = "  (" + to_string(dis) + ", " + to_string(theta) + ")";
      putText(buffer, info, center, cv::FONT_HERSHEY_DUPLEX, 0.3, Scalar(0,0,0), 0.5);
      cout<<" Goal in sight!"<<endl;

    }
  }
  cv::imshow("view", buffer);  //show the image with a window
  //========visualize=====
  //cv::imshow("hsv_red", hsv_red); cv::imshow("hsv_green", hsv_green);
  //cv::imshow("edges_red", edges_red); cv::imshow("edges_green", edges_green);
  //cv::imshow("wall", wall);
  //======================
  

  
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
  ball_detect(); //proceed ball detection
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ball_detect_node_tB"); //init ros node
  ros::NodeHandle nh; //create node handler
  image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
  image_transport::Subscriber sub = it.subscribe("/kinect_rgb", 1, imageCallback); //create subscriber

  pub = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher
  pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);

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

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);

}

vector<float> getBallPos(float x, float y,float r){
    //===============================================================
    //(X,Y,Z) --> ball position in camera coordinate 
    float ratio, X, Y, Z, dis, theta;
    ratio = R/r;
    X = ratio*(x-cx); Y = ratio*(y-cy); Z = ratio*f;

    // (dis,theta,k) --> ball position in polar coordiate
    dis = hypot(X,Z);            
    dis = truncf(dis * 1000) / 1000;
    theta = atan(X/Z);           
    
    vector<float> pos;
    pos.push_back(dis);
    pos.push_back(theta);
    
    return pos;
    //==============================================================
}

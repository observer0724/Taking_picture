/*test the segmentation method in a vedio*/

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <time.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <vector>
#include <iostream>
#include<string>

#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>

#include <std_msgs/Float64.h>

#include <stdlib.h>
#include <stdio.h>
#include "opencv/cv.hpp"
using namespace cv;

bool freshImage;
bool freshCameraInfo;

using namespace std;

// double weight_data;

// updates the local image.
void newImageCallback(const sensor_msgs::ImageConstPtr& msg, cv::Mat* outputImage)
{
	cv_bridge::CvImagePtr cv_ptr;
  ROS_INFO("IN THE CALLBACK");
    try
    {
       //cv::Mat src =  cv_bridge::toCvShare(msg,"32FC1")->image;
       //outputImage[0] = src.clone();
        ros::Duration(3).sleep();
    	cv_ptr = cv_bridge::toCvCopy(msg);
    	outputImage[0] = cv_ptr->image;
       freshImage = true;

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg ->encoding.c_str());
    }

}


// void scalerCallback(const std_msgs::Float64& weight)
// {
//   ROS_INFO("received value is: %f",weight.data);
//   weight_data = weight.data;
//   //really could do something interesting here with the received data...but all we do is print it
// }

 // This node takes as an input, the rectified camera image and performs stereo calibration.
int main(int argc, char** argv)
{
	ROS_INFO("---- In main node -----");

	ros::init(argc, argv, "davinci_test");

	ros::NodeHandle nh;

	ros::Rate timer(1.0);

	ROS_INFO("---- Initialized ROS -----");

  freshCameraInfo = false;
  freshImage = false;

	// get image size from camera model
    // initialize segmented images
  ROS_INFO("???");
    Mat rawImage_left = cv::Mat::zeros(640, 920, CV_32FC1);
    Mat rawImage_right = cv::Mat::zeros(640, 920, CV_32FC1);


    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub_l = it.subscribe("/davinci/left_camera/image_raw", 1,
      boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left)));
    image_transport::Subscriber img_sub_r = it.subscribe("/davinci/right_camera/image_raw", 1,
      boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

      Mat seg_left;
    Mat seg_right;
    ROS_INFO("Start image saving");
    // ros::Subscriber scaler_sub= nh.subscribe("scaler", 1, scalerCallback);
    int i = 0;
    char temp[10];
    string folder = "/home/observer0724/Desktop/";
    string name;
    while (i<10)
    {
        ros::spinOnce();

    	if (freshImage){
        sprintf(temp,"%d",i);
        string file(temp);
        name = folder + file +".png";

        imwrite(name,rawImage_left);
        i++;
    		freshImage = false;
    	}
      else{
        ROS_INFO("Image not reached");
      }
      timer.sleep();


        ros::spinOnce();
	}


}

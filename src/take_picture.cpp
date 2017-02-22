#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <cwru_davinci_traj_streamer/trajAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <time.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <vector>
#include <iostream>
#include <string>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <stdlib.h>
#include <stdio.h>
#include "opencv/cv.hpp"
using namespace cv;

bool freshImage;
bool freshCameraInfo;
using namespace std;
// void scalerCallback(const std_msgs::Float64& weight)
// {
//   ROS_INFO("received value is: %f",weight.data);
//   weight_data = weight.data;
//   //really could do something interesting here with the received data...but all we do is print it
// }

bool g_server_goal_completed= false;

void doneCb(const actionlib::SimpleClientGoalState& state,
        const cwru_davinci_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
    g_server_goal_completed = true;
}

double weight_data = 9.0;

void scalerCallback(const std_msgs::Float64& weight)
{
  ROS_INFO("received value is: %f",weight.data);

  weight_data = weight.data;
}


void newImageCallback(const sensor_msgs::ImageConstPtr& msg, cv::Mat* outputImage)
{
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
    	cv_ptr = cv_bridge::toCvCopy(msg);
    	outputImage[0] = cv_ptr->image;
       freshImage = true;

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg ->encoding.c_str());
    }

}






int main(int argc, char **argv) {
	//Set up our node.
	ros::init(argc, argv, "take_picture");
	ros::NodeHandle nh;



  Mat rawImage_left = cv::Mat::zeros(640, 920, CV_8UC3);
  Mat rawImage_right = cv::Mat::zeros(640, 920, CV_8UC3);
  Mat save_image = cv::Mat::zeros(640, 920, CV_8UC3);


  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub_l = it.subscribe("/davinci_endo/left/image_raw", 1,
    boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left)));
  image_transport::Subscriber img_sub_r = it.subscribe("/davinci_endo/right/image_raw", 1,
    boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

    Mat seg_left;
  Mat seg_right;
  // cv::cvtColor(rawImage_left, save_image, CV_XYZ2RGB);



	std::string fname;


	trajectory_msgs::JointTrajectory track;

	trajectory_msgs::JointTrajectoryPoint default_position;
	default_position.positions.resize(14);

	default_position.positions[0] = -0.66709;
	default_position.positions[1] = -0.0507692;
	default_position.positions[2] = 0.201701;
	default_position.positions[3] = 1.25122;
	default_position.positions[4] = -0.0885405;
	default_position.positions[5] = 0.0329113;
	default_position.positions[6] = 0.15338;
	default_position.positions[7] = 0.7;
	default_position.positions[8] = 0.1;
	default_position.positions[9] = 0.186111;
	default_position.positions[10] = 0.540371;
	default_position.positions[11] = -1;
	default_position.positions[12] = -0.0711621;
	default_position.positions[13] = -0.163507;
	default_position.time_from_start = ros::Duration(5.0);

	track.points.push_back(default_position);
	cwru_davinci_traj_streamer::trajGoal tstart;
	tstart.trajectory = track;



	double wait_time_1 = 2.0;
	ros::Rate naptime(2.0);


	// ros::Subscriber scaler_sub= nh.subscribe("scaler", 1, scalerCallback);
	track.header.stamp = ros::Time::now();

	//Add an ID.
	cwru_davinci_traj_streamer::trajGoal tgoal;
	tgoal.trajectory = track;

	srand(time(NULL));
	tgoal.traj_id = rand();

	//Locate and lock the action server
	actionlib::SimpleActionClient<
	  cwru_davinci_traj_streamer::trajAction
	> action_client("trajActionServer", true);
	bool server_exists = action_client.waitForServer(ros::Duration(5.0));
	// something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
	ROS_INFO("Waiting for server: ");
	while (!server_exists && ros::ok()) {
	  server_exists = action_client.waitForServer(ros::Duration(5.0));
	  ROS_WARN("Could not connect to server; retrying...");
	}
	ROS_INFO("SERVER LINK LATCHED");

	//Send our message:
	ROS_INFO("Sending trajectory with ID %u", tgoal.traj_id);

	ros::Subscriber scaler_sub= nh.subscribe("scale", 1, scalerCallback);
  //ROS_INFO("%f",weight.data);
  int pic_num = 0;
  char temp[16];
  string folder = "/home/dvrk/Desktop/pictures/";
  string name;
  double pos_11 = -1;
  for (int i=0; i<10; i++){
		tgoal.trajectory = track;
		
		pos_11 += 0.2;
		tgoal.trajectory.points[0].positions[11] = pos_11;
		ROS_INFO_STREAM("LOOP i is: " << i);
		ros::spinOnce();
	    while (weight_data <= 2.5){

			g_server_goal_completed= false;
			action_client.sendGoal(tgoal,&doneCb);

			while(!g_server_goal_completed){
				doneCb;
				ros::Duration(2).sleep();
	      		ROS_INFO("STILL MOVING");
	      // wait_time_1 += 2.0;
			}
			ros::spinOnce();
	    	ROS_INFO("Taking a picture");
	    	ros::Duration(2).sleep();
		    if (freshImage){
		      sprintf(temp,"%d",pic_num);
		      string file(temp);
		      if (weight_data>0.1){
		        name = folder+"touch/"+file+".png";
		      }
		      else{
		        name = folder+"not_touch/"+file+".png";
		      }
		      imwrite(name,rawImage_left);
		      freshImage = false;
		      pic_num ++;
		    }
	    	ROS_INFO("Picture taken");
			tgoal.trajectory.points[0].positions[9] += 0.002;

			tgoal.trajectory.points[0].time_from_start = ros::Duration(wait_time_1);
	    	tstart.trajectory.points[0].time_from_start = ros::Duration(wait_time_1+2.0);
			tgoal.traj_id = rand();
			// ros::Subscriber scaler_sub= nh.subscribe("scale", 1, scalerCallback);
	    	ROS_INFO("%f",weight_data);
		}
  		action_client.sendGoal(tstart,&doneCb);
  		g_server_goal_completed= false;
		while(!g_server_goal_completed){
			doneCb;
			ros::Duration(2).sleep();
		    ROS_INFO("STILL MOVING");
		      // wait_time_1 += 2.0;
		}
	}
	//Wait for it to finish.
	while(!action_client.waitForResult(ros::Duration(wait_time_1 + 4.0)) && ros::ok()){
	  ROS_WARN("CLIENT TIMED OUT- LET'S TRY AGAIN...");
	  //Could add logic here to resend the request or take other actions if we conclude that
	  //the original is NOT going to get served.
	}
	//Report back what happened.
	ROS_INFO(
	  "Server state is %s, goal state for trajectory %u is %i",
	  action_client.getState().toString().c_str(),
	  action_client.getResult()->traj_id,
	  action_client.getResult()->return_val
	);

	//This has to do with the intermittent "Total Recall" bug that breaks the trajectory interpolator
	//If you see this appear in your execution, you are running the program too soon after starting Gazebo.
	if(action_client.getState() ==  actionlib::SimpleClientGoalState::RECALLED){
	  ROS_WARN("Server glitch. You may panic now.");
	}

	return 0;
}

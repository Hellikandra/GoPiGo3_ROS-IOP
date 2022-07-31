// C/C++ libraries
#include <iostream>
#include <stdio.h>

// Other libraries which need to be installed
#include <opencv2/highgui/highgui.hpp>

// ROS libaries interface
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

void rospigo3BatteryCallback (const std_msgs::Float64::ConstPtr& battery_level)
{
  ROS_INFO("I heard: [%f]", battery_level->data);
}

void rospigo3GPSCallback (const std_msgs::Float64::ConstPtr& gps_location)
{
  ROS_INFO("GoPiGo 1 is at :%f",gps_location->data);
}

void imageCallback (const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    ROS_INFO("We received an image from cv_bridge");
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(25);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
int main (int argc, char ** argv)
{
  ROS_INFO("OCU reception");

  ros::init(argc, argv, "rosocu");
  ros::NodeHandle n;

  cv::namedWindow("view");

  ros::Subscriber sub = n.subscribe("ropigo3_battery", 1000, rospigo3BatteryCallback);
  ros::Subscriber sub1 = n.subscribe("ropigo_gps", 1000, rospigo3GPSCallback);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_it = it.subscribe("camera/image", 1, imageCallback);

  ros::spin();

  cv::destroyWindow("view");
  return 0;
}

// C/C++ libraries
#include <iostream>
#include <stdio.h>

// GoPiGo3 library interface
#include <GoPiGo3.h>

// ROS libraries interface
#include <ros/ros.h>
#include <std_msgs/Float64.h>

GoPiGo3 GPG;

int main (int argc, char ** argv)
{

  ROS_INFO("test %d", INPUT_DIGITAL);

  if (GPG.detect(false) == ERROR_NONE)
   ROS_INFO("GoPiGo3 not detected");

  ROS_INFO("GoPiGo3 Firmware Version:");
  ROS_INFO("GoPiGo3 Board Version:");

  ros::init(argc, argv, "ropigo3");

  ROS_INFO("Exit.");

  return 0;
}

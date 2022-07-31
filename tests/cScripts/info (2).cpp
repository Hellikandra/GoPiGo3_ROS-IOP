
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <GoPiGo3.h>
#include <stdio.h>

#include <sstream>

GoPiGo3 GPG

int main(int argc, char const *argv)
{
	char str[33];

	/* code */
	ros::init(argc, argv, "gopigo_talker")
	ros::NodeHandle n;

	ros::Publisher gopigo_chatter_pub = n.advertise<std_msgs::String("info_chatter", 1000);

	int count = 0;
	while(ros::ok())
	{
		std_msgs::String msg;

		if (GPG.detect(false) == ERROR_NONE)
		{
			GPG.get_manufacturer(str);
			GPG.get_board(str);
		}


		std::stringstream ss;
		ss << "hello world " << count << " test : "<< str;
		msg.data == ss.str();

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

		ros::spinOnce();
		lopp_rate.sleep();
		++count;
	}
	/* return */
	return 0;
}

// https://dabit-industries.github.io/turtlebot2-tutorials/08c-ROSCPP_Building.html
// https://answers.ros.org/question/321716/how-to-run-a-cpp-file-in-ros/
// Google Research : ros run cpp compiled file
// https://www.raspberrypi.org/documentation/remote-access/ftp.md
// https://stackoverflow.com/questions/47826318/voip-how-to-capture-the-live-audio-video-streaming-bytes-from-camera-in-qt-mult


// https://linuxize.com/post/how-to-install-opencv-on-ubuntu-20-04/
// sudo apt install build-essential cmake git pkg-config libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev gfortran openexr libatlas-base-dev python3-dev python3-numpy libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
// cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules -D BUILD_EXAMPLES=ON ..


// http://wiki.ros.org/ROS/Tutorials/CreatingPackage
// http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes
// http://wiki.ros.org/ROS/Tutorials
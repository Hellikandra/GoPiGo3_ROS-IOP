/* C/C++ Libraries */
#include <ctime>
#include <errno.h>      // Error integer and strerror() function
#include <fcntl.h>      // Containe file controls like O_RDWR
#include <iostream>
#include <math.h>
#include <signal.h>    // for catching exit signals
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <termios.h>    // Contains POSIX terminal control definitions
#include <thread>
#include <unistd.h>     // write(), read(), close()
//#include <sys/ioctl.h>  // Used for TCGETS2/TCSETS2, which is required for custom baud rates

/* Other Libraries who need to be installed */
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>

/* ROS Package dependencies */
#include <GoPiGo3.h>

/* ROS Libraries interface */
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <image_transport/image_transport.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>

struct gps_gga{
  std::string msg_id;
  std::string utc_pos;
  std::string latitude;
  std::string ns_indicator;
  std::string longitude;
  std::string ew_indicator;
  std::string position_fix_indicator;
  std::string satellites_used;
  std::string hdop;
  std::string msl_altitude;
  std::string units1;
  std::string geoids_separation;
  std::string units2;
  std::string age_of_diff_corr;
  std::string diff_ref_station_id;
  std::string checksum;
  std::string cr_lf;
};

GoPiGo3 GPG;
#define WHEEL_BASE_WIDTH 117
#define WHEEL_DIAMTER 66.5
#define WHEEL_BASE_CIRCUMFERENCE (WHEEL_BASE_WIDTH * M_PI)
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMTER * M_PI)

/* Thread : rapsicam acquistion and send to ROS Node */
/* Currently managed by a RTSP server via syscall */

void pRaspiCamPublisherThread (ros::NodeHandle n) 
{
  // /* Variables Declaration */
  // cv::Mat image;
  // raspicam::RaspiCam_Cv camera;
  // sensor_msgs::ImagePtr msg;

  // /* Pre-Condition */
  // /* Implementation */
  // camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
  // std::cout << "Opening Camera ..." << std::endl;

  // if(!camera.open())
  // {
  //   ROS_ERROR("Error opening the raspicam");
  //   //std::cerr << "Error opening the raspicam" << std::endl;
  //   return;
  // }
  // image_transport::ImageTransport rpicam_it(n);
  // image_transport::Publisher rpicam_pub = rpicam_it.advertise("camera/stream", 1);

  // /* Thread Loop */
  // while (ros::ok())
  // {
  //   camera.grab();
  //   camera.retrieve(image);
  //   msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
  //   ROS_INFO_THROTTLE(1, "PUBLISH : raspicam"); // print every 1 seconds
  //   rpicam_pub.publish(msg);
  //   ros::spinOnce();
  //   cv::waitKey(25); // 25 ms
  // }
  // /* Post-Condition */
  // camera.release();
  // Temporary Implementation

  ROS_INFO("Raspicam is set by syscall as v4l2rtspserver");
  // remember to launch modprobe before
  system("v4l2rtspserver -W 640 -H 480 -F 15 -P 8554 /dev/video0");
  while(ros::ok())
  {
    ros::spinOnce();
    cv::waitKey(25); // 25 ms
  } 
}

/* Thread : Groovy GPS module acquisition and sent to ROS Node */
void pGroovyGpsPublisherThread (ros::NodeHandle n)
{
  /* Variables Declaration */
  char            read_buffer[256];
  char            tmp_buffer[256];
  int             num_bytes;
  int             index_min, index_max;
  int             serial_port = open("/dev/ttyS0", O_RDWR);
  struct          termios tty;
  struct          gps_gga groovy_gga;
  char            nmea_gpgga[5] = {'G','P','G','G','A'};
  size_t          pos = 0;
  int             iStIdx_groovy_gga = 1;
  std::string     nmea_delimiter = ",";
  std::string     str_subStr_token;
  /* Pre-Condition */
  memset(&groovy_gga, 0, sizeof(gps_gga));
  // just for debug - set as initial configuration until we received data from Groovy
  groovy_gga.latitude = "45.55";
  groovy_gga.longitude = "5.15";
  groovy_gga.msl_altitude = "110.0";
  if (serial_port < 0)
  {
    ROS_ERROR("Error %i from open: %s", errno, strerror(errno));
    return;
  }
  if (tcgetattr(serial_port, &tty) != 0)
  {
    ROS_ERROR("Error %i from tcgetattr: %s", errno, strerror(errno));
    return;
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 0;
  cfsetspeed(&tty, B9600);  // in/out directly

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
  {
     ROS_ERROR("Error %i from tcsetattr: %s\n", errno, strerror(errno));
     return;
  }

  /* Implementation */
  ros::Rate loop_rate(0.2f); // 0.2 Hz Frequency (every 5 seconds)
  ros::Publisher groovy_gps_pose = n.advertise<sensor_msgs::NavSatFix>("/geographic/GroovyGPS", 10,true);
  //ros::Publisher groovy_gps_pose = n.advertise<sensor_msgs::NavSatFix>("/fix", 10,true);
  sensor_msgs::NavSatFix groovy_geo_pose;

  /* Thread Loop */
  while(ros::ok())
  {
    memset(&read_buffer, '\0', sizeof(read_buffer));
    num_bytes = read(serial_port, &read_buffer, sizeof(read_buffer));
    if (num_bytes < 0)
    {
      ROS_WARN("Error serial port reading : %s", strerror(errno));
    }
    ROS_DEBUG_STREAM("Read " << num_bytes << " bytes. Received message : \n" << read_buffer);
    for (int i = 0; i < num_bytes;i++)
    {
      if (read_buffer[i] == '$')
      {
        index_min = i;
        char nmea_decoder[5] = {read_buffer[i+1], read_buffer[i+2], read_buffer[i+3], read_buffer[i+4], read_buffer[i+5]};

        if (strcmp(nmea_decoder,nmea_gpgga) > 12)
        {
          
          for (int j = i; j < strlen(read_buffer) - i; j++)
          {
            if (read_buffer[j] == '\r' && read_buffer[j+1] == '\n')
            {
              //std::cout << "we found crlf end char" << std::endl;
              index_max = j;
              break;
            }
          }
          if (index_max > 256 || index_max < index_min)
          {
            ROS_ERROR("Error in GPS module message");
            break;
          }
          strncpy(tmp_buffer, read_buffer + i, index_max - index_min);
          std::string str_tmp_buffer(tmp_buffer);

          pos = 0;
          iStIdx_groovy_gga = 1;
          while ((pos = str_tmp_buffer.find(nmea_delimiter)) != std::string::npos)
          {
            str_subStr_token = str_tmp_buffer.substr(0,pos);
            str_tmp_buffer.erase(0, pos + nmea_delimiter.length());
            if (str_subStr_token != "")
            {
              switch(iStIdx_groovy_gga)
              {
                case 1:  groovy_gga.msg_id = str_subStr_token;                  break;
                case 2:  groovy_gga.utc_pos = str_subStr_token;                 break;
                case 3:  groovy_gga.latitude = str_subStr_token;                break;
                case 4:  groovy_gga.ns_indicator = str_subStr_token;            break;
                case 5:  groovy_gga.longitude = str_subStr_token;               break;
                case 6:  groovy_gga.ew_indicator = str_subStr_token;            break;
                case 7:  groovy_gga.position_fix_indicator = str_subStr_token;  break;
                case 8:  groovy_gga.satellites_used = str_subStr_token;         break;
                case 9:  groovy_gga.hdop = str_subStr_token;                    break;
                case 10: groovy_gga.msl_altitude = str_subStr_token;            break;
                case 11: groovy_gga.units1 = str_subStr_token;                  break;
                case 12: groovy_gga.geoids_separation = str_subStr_token;       break;
                case 13: groovy_gga.units2 = str_subStr_token;                  break;
                case 14: groovy_gga.age_of_diff_corr = str_subStr_token;        break;
                case 15: groovy_gga.diff_ref_station_id = str_subStr_token;     break;
                case 16: groovy_gga.checksum = str_subStr_token;                break;
                case 17: groovy_gga.cr_lf = str_subStr_token;                   break;
                default: ROS_WARN("Out of index to fill structure");
              };
            }
            iStIdx_groovy_gga +=1;
          }
        }      
      }
    }
    // Due to the fact that the GPS module send a wrong message but we already filled the structure, we can always send the actual position of the robot.
    // in the case of a part of the message is missing but we can catch a part of the message, a part of the GPS coordinates can changed and still get the last successuf part of the message.
    if ((groovy_gga.latitude != "") && (groovy_gga.longitude != "") && (groovy_gga.msl_altitude != ""))
    {
      // Currently we only provide GPS coordinate to the IOP through fkie_iop_global_pose_sensor
      groovy_geo_pose.latitude = std::stod (groovy_gga.latitude);
      groovy_geo_pose.longitude = std::stod (groovy_gga.longitude);
      groovy_geo_pose.altitude = std::stod (groovy_gga.msl_altitude);
      
      ROS_INFO_STREAM("PUBLISH : GPS Information : Lat : " << groovy_gga.latitude << "\tLong : " << groovy_gga.longitude << "\tAlt : " << groovy_gga.msl_altitude);
      groovy_gps_pose.publish(groovy_geo_pose);
    }
    else
    {
      ROS_WARN("GPS module does not provide info.");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  /* Post-Condition */
  close(serial_port);
}

/* Thread : GoPiGo3 status acquisition and sent to ROS Node */
/* Currently I do not see a specific IOP node where I can put some information about that */
void pGoPiGo3StatusPublisherThread (ros::NodeHandle n)
{
  /* Variables Declaration */
  char str_buf[33];
  diagnostic_msgs::DiagnosticArray  dmsg;
  diagnostic_msgs::DiagnosticStatus diag;

  diagnostic_msgs::KeyValue gopigo3_manufacturer;
  diagnostic_msgs::KeyValue gopigo3_board;
  diagnostic_msgs::KeyValue gopigo3_serial_number;
  diagnostic_msgs::KeyValue gopigo3_hardware_version;
  diagnostic_msgs::KeyValue gopigo3_firmware_version;
  diagnostic_msgs::KeyValue gopigo3_battery_voltage;
  diagnostic_msgs::KeyValue gopigo3_5v_voltage;
  
  float voltage_battery = 0.0f;
  float voltage_5v      = 0.0f;
  /* Pre-Condition */
  if (GPG.detect(false) != ERROR_NONE)
  {
    ROS_ERROR("Enable to communicate with GoPiGo3");
  }
  /* Implementation */
  ros::Publisher diag_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics/gopigo3_info", 10, true);
  ros::Rate loop_rate(1); // 1 Hz

  /* Thread Loop */
  while(ros::ok())
  {
    dmsg.header.stamp = ros::Time::now();
    diag.level   = diagnostic_msgs::DiagnosticStatus::OK;
    diag.name    = ros::this_node::getName();
    diag.message = "Get generic information.";
    
    GPG.get_manufacturer(str_buf);
    ROS_INFO_THROTTLE(60, "Manufacturer     : %s", str_buf);
    gopigo3_manufacturer.key = "Manufacturer";
    gopigo3_manufacturer.value = str_buf;
    
    GPG.get_board(str_buf);
    ROS_INFO_THROTTLE(60,"Board            : %s", str_buf);
    gopigo3_board.key   = "Board";
    gopigo3_board.value = str_buf;
    
    GPG.get_id(str_buf);
    ROS_INFO_THROTTLE(60, "Serial Number    : %s", str_buf);
    gopigo3_serial_number.key   = "Serial Number";
    gopigo3_serial_number.value = str_buf;
    
    GPG.get_version_hardware(str_buf);
    ROS_INFO_THROTTLE(60, "Hardware version : %s", str_buf);
    gopigo3_hardware_version.key   = "Hardware version";
    gopigo3_hardware_version.value = str_buf;

    GPG.get_version_firmware(str_buf);
    ROS_INFO_THROTTLE(60, "Firmware version : %s", str_buf);
    gopigo3_firmware_version.key   = "Firmware version";
    gopigo3_firmware_version.value = str_buf;
    
    voltage_battery = GPG.get_voltage_battery();
    ROS_INFO_THROTTLE(60, "Battery voltage  : %.3f", voltage_battery);
    gopigo3_battery_voltage.key   = "Battery voltage";
    gopigo3_battery_voltage.value = std::to_string(voltage_battery);

    voltage_5v= GPG.get_voltage_5v();
    ROS_INFO_THROTTLE(60, "5V voltage       : %.3f", voltage_5v);
    gopigo3_5v_voltage.key   = "5V voltage";
    gopigo3_5v_voltage.value = std::to_string(voltage_5v);    

    diag.values.push_back(gopigo3_manufacturer);
    diag.values.push_back(gopigo3_board);
    diag.values.push_back(gopigo3_serial_number);
    diag.values.push_back(gopigo3_hardware_version);
    diag.values.push_back(gopigo3_firmware_version);
    diag.values.push_back(gopigo3_battery_voltage);
    diag.values.push_back(gopigo3_5v_voltage);

    dmsg.status.push_back(diag);
    diag_pub.publish(dmsg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  /* Post-Condition */
}
/* Thread : GoPiGo3 motors status acquisition and sent to ROS Node */
void pGoPiGo3MotorsPublisherThread (ros::NodeHandle n) 
{
  /* Variables Declaration */
  int32_t EncoderLeft_tmp = 0;
  int32_t EncoderRight_tmp = 0;
  /* Pre-Condition */
  ROS_INFO("gopigo3_motor_status_callback");
  GPG.detect();
  // if (!GPG.detect())
  // {
  //   ROS_ERROR("GoPiGo3 general failure");
  //   return;
  // }
  // Reset the encoders
  ROS_INFO("Reset Encoders");
  GPG.offset_motor_encoder(MOTOR_LEFT, GPG.get_motor_encoder(MOTOR_LEFT));
  GPG.offset_motor_encoder(MOTOR_RIGHT, GPG.get_motor_encoder(MOTOR_RIGHT));
  
  /* Implementation */
  ros::Rate sleep_rate(50);
  //ROS_INFO("Start loop motor thread");
  /* Thread Loop */
  while(ros::ok())
  {
    // Read the encoders
    int32_t EncoderLeft = GPG.get_motor_encoder(MOTOR_LEFT);
    int32_t EncoderRight = GPG.get_motor_encoder(MOTOR_RIGHT);

    // Use the encoder value from the left motor to control the position of the right motor
    GPG.set_motor_position(MOTOR_RIGHT, EncoderLeft);

    // Display the encoder values
    if ((EncoderLeft != EncoderLeft_tmp) && (EncoderRight != EncoderRight_tmp))
    {
      ROS_INFO_STREAM("Encoder Left: " << EncoderLeft << " Right: " << EncoderRight);
    }
    else
    {
      ROS_INFO_STREAM_THROTTLE(5, "Wheel does not move Encoder Left: " << EncoderLeft << ":" << EncoderLeft_tmp << " Right: " << EncoderRight << ":" << EncoderRight_tmp);
    }
    //ROS_INFO("Encoder readed");
    
    // task before sleep
    EncoderLeft_tmp = EncoderLeft;
    EncoderRight_tmp = EncoderRight; 

    // Delay for 20ms
    ros::spinOnce();  // will call all the callbacks waiting to be called at that point in time.
    sleep_rate.sleep();
  }
  /* Post-Condition */
  GPG.reset_all(); // Reset everything so there are no run-away motors

}

void pGoPiGo3LedsPublisherThread(ros::NodeHandle n)
{
  /* Variables Declaration */
  /* Pre-Condition */
  /* Implementation */
  /* Thread Loop */
  /* Post-Condition */  
}

void aSyncGPScallback(const sensor_msgs::NavSatFix::ConstPtr& gps_coordinate)
{
  /* Variables Declaration */
  const double latitude  = gps_coordinate->latitude;
  const double longitude = gps_coordinate->longitude;
  const double altitude  = gps_coordinate->altitude;

  /* Pre-Condition */
  // we need to check if we have our own coordinate
  // compare it and provide a azimuth and distance to travel.

  /* Implementation */
  ROS_INFO_STREAM("GPS coordinate received : latitude : " << latitude << ";\tlongitude : " << longitude << ";\taltitude : " << altitude);
  
  // get the starting position of each motor
  int32_t StartPositionLeft  = GPG.get_motor_encoder(MOTOR_LEFT);
  int32_t StartPositionRight = GPG.get_motor_encoder(MOTOR_RIGHT);

  // the disrance in mm that each wheel need to travel
  int32_t WheelTravelDistance = (WHEEL_BASE_CIRCUMFERENCE * 10) / 360;
  // the number of degrees of each wheel need to turn
  int32_t WheelTurnDegrees = (WheelTravelDistance / WHEEL_CIRCUMFERENCE) * 360;
  // Limit the speed
  GPG.set_motor_limits( MOTOR_RIGHT, 100, 100);
  GPG.set_motor_limits(MOTOR_LEFT, 100, 100);

  // Set each motor target
  GPG.set_motor_position(MOTOR_LEFT, (StartPositionLeft + WheelTurnDegrees));
  GPG.set_motor_position(MOTOR_RIGHT, (StartPositionRight - WheelTurnDegrees));

  /* Post-Condition */
  sleep(3);
  GPG.set_motor_power(MOTOR_LEFT, 0);
  GPG.set_motor_power(MOTOR_RIGHT, 0);
  //GPG.reset_all();
}

void aSyncGlobalSTPTcallback(const nav_msgs::Path::ConstPtr& path_data)
{
  /* Variables Declaration */
  /* Pre-Condition */

  /* Implementation */
  ROS_WARN("Steerpoint information received");
  
  // decode information from path_data.

  /* Post-Condition */
  sleep(3);

}

/* main function */
int main (int argc, char **argv)
{
  /* Variables Declaration */
  

  /* Pre-Condition */
  ros::init(argc, argv, "ropigo3");
  ros::NodeHandle n;

  /* Implementation */
  /* Thread for each publisher */
  // std::thread thRaspiCamPublisher(pRaspiCamPublisherThread, n);
  std::thread thGroovyGpsPublisher(pGroovyGpsPublisherThread, n);
  // std::thread thGoPiGo3StatusPublisher(pGoPiGo3StatusPublisherThread, n);
  std::thread thGoPiGo3MotorsPublisher(pGoPiGo3MotorsPublisherThread,n);
  std::thread thGoPiGo3LedsPublisher(pGoPiGo3LedsPublisherThread, n);
  ros::Rate sleep_rate(10); // 10 Hz

  ros::Subscriber aSyncGPS = n.subscribe("cmd_fix", 10, aSyncGPScallback); // do we need to check fix value ?
  ros::Subscriber aSyncGlobalSTPT = n.subscribe("cmd_global_waypoints", 10, aSyncGlobalSTPTcallback); // fkie_iop_global_waypoint_list_driver
  while(ros::ok())
  {
    // ROS Subscribe
    ros::spinOnce();  // will call all the callbacks waiting to be called at that point in time.
    sleep_rate.sleep();
  }
  /* Post-Condition */
  // thRaspiCamPublisher.join();
  thGroovyGpsPublisher.join();
  // thGoPiGo3StatusPublisher.join();
  thGoPiGo3MotorsPublisher.join();
  thGoPiGo3LedsPublisher.join();
  ROS_WARN("End of GoPiGo3 IOP/ROS-Bridge.");
  return 0;
}

/* End of File */


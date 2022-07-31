// C/C++ libraries
#include <ctime>
#include <iostream>
#include <stdio.h>
#include <string.h>

#include <fcntl.h>  // Contains file controls like O_RDWR
#include <errno.h>  // Error integer and strerror() function
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <thread>
//#include <sys/ioctl.h> // Used for TCGETS2/TCSETS2, which is required for custom baud rates

// Other libraries who need to be installed
#include <opencv2/opencv.hpp>

// GoPiGo3 library interface
#include <GoPiGo3.h>

// Raspberry-Pi libraries interface
#include <raspicam/raspicam_cv.h>

// ROS libraries interface
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

GoPiGo3 GPG;

class thread_obj {
public:
  void operator()(int x)
  {
    ROS_INFO("message from operator in class thread_obj\n");
    for (int i = 0; i < x; i++)
    {
      std::cout << "Thread using function object as callable\n";
      usleep(10000);
    }
  }
};


void foo(int z)
{
  // do something ?
  ROS_INFO("message from foo");
  for (int i = 0; i < z; i++)
  {
    std::cout << "Thread using function pointer as callable\n";
    usleep(10000);
  }
}


std_msgs::Float64 get_gps_location(void)
{
 std_msgs::Float64 latitude;
 latitude.data = 59.6365;

 return latitude ;
}

int main (int argc, char ** argv)
{
  // Variables Declaration
  char str[33];
  time_t timer_begin, timer_end;
  raspicam::RaspiCam_Cv Camera;
  cv::Mat image;
  sensor_msgs::ImagePtr msg;
  int nCount = 100;
  int serial_port = open("/dev/ttyS0", O_RDWR);
  // check for errors
  if (serial_port < 0)
  {
    printf("Error %i from open: %s\n", errno, strerror(errno));
  }
  // Create new termios struct, we call it 'tty' for convention
  // No need for "= {0}" at the end as we'll immediately write the existing
  // config to this struct
  struct termios tty; // old one used with termios.h
  // struct termios2 tty; // pretty similar but some other things are added.

  // Read in existing settings, and handle any error
  // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
  // must have been initialized with a call to tcgetattr() overwise behaviour
  // is undefined
  if(tcgetattr(serial_port, &tty) != 0)
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  // Control Modes (c_cflag)
  // -- PARENB (Parity)
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  // tty.c_cflag |= PARENB;  // Set parity bit, enabling parity

  // -- CSTOPB (Num. Stop Bits
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  //tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication

  // -- Number Of Bits Per Byte
  tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
  //tty.c_cflag |= CS5; // 5 bits per byte
  //tty.c_cflag |= CS6; // 6 bits per byte
  //tty.c_cflag |= CS7; // 7 bits per byte
  tty.c_cflag |= CS8; // 8 bits per byte (most common)

  // -- Flow Control (CRTSCTS)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  //tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control

  // -- CREAD and CLOCAL
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  // Local Modes (c_lflag
  // -- Disabling Canonical Mode
  tty.c_lflag &= ~ICANON;

  // -- Echo
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo

  // -- Disable Signal Chars
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  // Input Modes (c_iflag)
  // -- Software Flow Control (IXOFF, IXON, IXANY)
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

  // -- Disabling Special Handling Of Bytes On Receive
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  // Output Modes (c_oflag)
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  //tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
  //tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

  //VMIN and VTIME (c_cc)
  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Baud Rate
  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B9600); // in
  cfsetospeed(&tty, B9600); // out
  cfsetspeed(&tty, B9600);  // in/out directly

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
  {
     printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  // Write to serial port
  //unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
  //write(serial_port, "Hello, world!", sizeof(msg));

  // Allocate memory for read buffer, set size according to your needs
  char read_buf [256];

  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s\n", strerror(errno));
      //return 1;
  }

  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
  // print it to the screen like this!)
  printf("Read %i bytes. Received message: %s\n", num_bytes, read_buf);

  //close(serial_port);

  std::cout << "Thread 1 and 2 and 3 operating independently" << std::endl;

  // this thread is launched by using function pointer as callable
  std::thread th1(foo,20);
  // this thread is launched by using function object as callable
  std::thread th2(thread_obj(), 20);

  // define a lambda Expression
  auto f = [](int x) {
      for (int i = 0; i < x ; i++)
      {
        std::cout << "Thread using lambda expression as callable\n";
        usleep(10000);
      }
  };
  // This thread is launched by using lambda expression as callable
  std:: thread th3(f, 20);

  // Wait for the threads to finish
  // Wait for the thread t1 to finish
  th1.join();

  // Wait for the thread t2 to finish
  th2.join();

  // Wait for the thread t3 to finish
  th3.join();

  Camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
  std::cout << "Opening Camera ..." << std::endl;

  if(!Camera.open())
  {
    std::cerr << "Error opening the camera" << std::endl;
  }
  else
  {
    std::cout << "Camera is present and open" << std::endl;
  }
  ROS_INFO("test %d", INPUT_DIGITAL);

  if (GPG.detect(false) == ERROR_NONE)
  {
    ROS_INFO("GoPiGo3 not detected");
    GPG.get_manufacturer(str);
    ROS_INFO("    Manufacturer     : %s", str);

    GPG.get_board(str);
    ROS_INFO("    Board            : %s", str);

    GPG.get_id(str);
    ROS_INFO("    Serial Number    : %s", str);

    GPG.get_version_hardware(str);
    ROS_INFO("    Hardware version : %s", str);

    GPG.get_version_firmware(str);
    ROS_INFO("    Firmware version : %s", str);

    ROS_INFO("    Battery voltage  : %.3f", GPG.get_voltage_battery());
    ROS_INFO("    5V voltage       : %.3f", GPG.get_voltage_5v());
  }
  else
  {
    ROS_INFO("Enable to communicate with GoPiGo3\n");
  }


  ros::init(argc, argv, "ropigo3");
  ros::NodeHandle n;

  ros::Publisher battery_pub = n.advertise<std_msgs::Float64>("ropigo3_battery", 1000);
  ros::Publisher gps_pub = n.advertise<std_msgs::Float64>("ropigo_gps", 1000);

  image_transport::ImageTransport rpicam_it(n);
  image_transport::Publisher rpicam_pub = rpicam_it.advertise("camera/image", 1);

  ros::Rate loop_rate(10);

  int count = 0;
  std::cout << "Capturing frames" << std::endl;

  while (ros::ok())
  {
   // ----- raspicam management
   Camera.grab();
   Camera.retrieve (image);
   // cv::imshow("Display window", image);
   msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
   rpicam_pub.publish(msg);
   cv::waitKey(25);

   // ----- Battery level management
   std_msgs::Float64 battery_level;

   battery_level.data = GPG.get_voltage_battery();
   ROS_INFO("battery level : %.3f", battery_level.data);

   battery_pub.publish(battery_level);

   // ----- SERIAL GPS catch information
   num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
   if (num_bytes < 0)
   {
     printf("Error reading: %s", strerror(errno));

   }
   printf("Read %i bytes. Received message: %s", num_bytes, read_buf);
   // ----- GPS Location management
   std_msgs::Float64 gps_location;

   gps_location = get_gps_location();
   ROS_INFO("GPS Location : %.3f", gps_location.data);

   gps_pub.publish(gps_location);

   ros::spinOnce();
   loop_rate.sleep();
   ++count;

  }
  close(serial_port);
  Camera.release();
  ROS_INFO("Exit.");

  return 0;
}


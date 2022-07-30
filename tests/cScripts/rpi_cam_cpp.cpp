#include <iostream>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>

int main (int argc, char** arv)
{
  cv::Mat image, output;
  // VideoCapture cap(CV_CAP_ANY);
  raspicam::RaspiCam_Cv cap;

  if (!cap.open() )
  {
    std::cout << "Could not initialize capturing ...\n";
    return -1;
  }
  while (1)
  {
    // cap >> ouput;

    cap.grab();
    cap.retrieve(output);

    imshow("Webcam input", output);
    char c = (char)cv::waitKey(10);
    if ( c == 27)
      break;
  }
  return 0;
}

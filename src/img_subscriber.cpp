#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  namedWindow("view");
  startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(argv[1], 2, imageCallback);
  ros::spin();
  destroyWindow("view");
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include <sstream>
#include <stdio.h>

#include <img_proc/Point.h>
#include <img_proc/EdgePosition.h>

using namespace cv;

class EdgeDetector
{
  int threshold1, threshold2, pos_id_;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher edge_pub_;

public:
  EdgeDetector(char*& sub_topic, char*& threshold1_str, char*& threshold2_str)
    : it_(nh_)
    , pos_id_(0)
  {
    std::stringstream ss;
    ss << threshold1_str;
    ss >> threshold1;
    ss << threshold2_str;
    ss >> threshold2;
    image_sub_ = it_.subscribe(sub_topic, 2, &EdgeDetector::imageCallback, this);
    image_pub_ = it_.advertise("edge_detector/output_video", 2);
    edge_pub_ = nh_.advertise<img_proc::EdgePosition>("edge_detector/edge_position", 2);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    Mat gray, blurImg;
    cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);
    blur(gray, blurImg, Size(3, 3));
    Canny(blurImg, cv_ptr->image, threshold1, threshold2);
    image_pub_.publish(cv_ptr->toImageMsg());
    publishEdgePosition(cv_ptr->image, msg->header.frame_id);
  }

  void publishEdgePosition(const Mat& image, const std::string frame_id)
  {
    img_proc::EdgePosition edgePos;
    for(int r = 0; r < image.rows; r++)
    {
      for(int c = 0; c < image.cols; c++)
      {
        if(image.at<uchar>(r, c) > 127)
        {
          img_proc::Point p;
          p.x = c;
          p.y = r;
          edgePos.data.push_back(p);
        }
      }
    }
    edgePos.header.stamp = ros::Time::now();
    edgePos.header.seq = pos_id_++;
    edgePos.header.frame_id = frame_id;
    edge_pub_.publish(edgePos);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "edge_detector");
  EdgeDetector ed(argv[1], argv[2], argv[3]);
  ros::spin();
  return 0;
}

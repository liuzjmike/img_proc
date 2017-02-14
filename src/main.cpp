#include <ros/ros.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "img_proc.h"

using std::vector;
using cv::Mat;

class Controller
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_;

public:
  Controller(char*& topic)
    : it_(nh_)
  {
    sub_ = it_.subscribe(topic, 2, &Controller::imageCallback, this);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    Mat edgeImg = ImageProcessor::detectEdge(cv_ptr->image);
    vector<int> edgePoints = ImageProcessor::locateEdge(edgeImg);
    for(int i = 0; i < edgePoints.size(); i += 2)
    {
      if(edgePoints[i+1] < 200)
      {
        std::cout << edgePoints[i] << " " << edgePoints[i+1] << std::endl;
      }
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_proc");
  Controller controller(argv[1]);
  ros::spin();
  return 0;
}

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/opencv.hpp"
#include "flann/flann.hpp"

#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

namespace img_proc
{

  cv::Mat detectEdge(const cv::Mat& image, const int loThresh, const int hiThresh);
  std::vector<int> locateEdge(const cv::Mat& edgeImage);
  flann::Index<flann::L2<int> > buildIndex(std::vector<int> features, const flann::IndexParams& params);
  float diff(const flann::Index<flann::L2<int> >& index, std::vector<int> queries, const float radius);

  class ImageProcessor
  {
  public:
    ImageProcessor(const flann::IndexParams& params = flann::KDTreeIndexParams(4));

    //edgeTopic should be joint_states
    void recordFeature(char*& imageTopic, char*& jointTopic, const int numRecords = 100,
      const int loThresh = 100, const int hiThresh = 160);
    float computeCost(char*&  imageTopic, char*&  edgeTopic);

  private:
    void featureImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void featureJointCallback(const sensor_msgs::JointStateConstPtr& msg);
    void queryImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void publishJointState();
    std::vector<int> convertImage(const sensor_msgs::ImageConstPtr& msg);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;
    ros::Subscriber joint_sub_;
    ros::Publisher joint_pub_;

    std::vector<flann::Index<flann::L2<int> > > indices_;
    std::vector<sensor_msgs::JointState> joint_states_;

    int total_, record_seq_, broadcast_seq_, lo_thresh_, hi_thresh_;
    float error_;
    bool record_flag_;

    flann::IndexParams params_;
  };
}

#endif

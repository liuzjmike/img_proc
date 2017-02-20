#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Trigger.h"
#include "img_proc/ComputeCost.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/opencv.hpp"
#include "flann/flann.hpp"

#include <thread>

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
    ImageProcessor(const flann::IndexParams& params = flann::KDTreeIndexParams(4), const int loThresh = 100, const int hiThresh = 160);

    void listenFeature(const std::string& imageTopic, const std::string& jointTopic);
    void stopListenFeature();

  private:
    bool triggerCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
    bool computeCost(img_proc::ComputeCost::Request& req, img_proc::ComputeCost::Response& res);
    void featureImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void featureJointCallback(const sensor_msgs::JointStateConstPtr& msg);
    void queryImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void publishJointState(const std::string& jointTopic);
    std::vector<int> convertImage(const sensor_msgs::ImageConstPtr& msg);
    bool samePosition(std::vector<double> position1, std::vector<double> position2);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;
    ros::ServiceServer record_service_, compare_service_;
    ros::Subscriber joint_sub_;
    ros::Publisher joint_pub_;

    std::vector<flann::Index<flann::L2<int> > > indices_;
    std::vector<sensor_msgs::JointState> joint_states_;

    int record_seq_, broadcast_seq_, lo_thresh_, hi_thresh_;
    float error_;
    bool image_flag_, joint_flag_;
    std::vector<double> position_;

    flann::IndexParams params_;

    const double threshold = 0.001;
    const std::string record_topic = "record_feature", compare_topic = "compute_cost";
  };
}

#endif

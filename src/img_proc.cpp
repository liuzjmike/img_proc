#include "img_proc.h"

using std::vector;
using std::string;
using cv::Mat;
using cv::Size;
using namespace flann;

namespace img_proc
{

  Mat detectEdge(const Mat& image, const int loThresh, const int hiThresh)
  {
    Mat gray, blurImg;
    cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    blur(gray, blurImg, Size(3, 3));
    Canny(blurImg, gray, loThresh, hiThresh);
    return gray;
  }

  vector<int> locateEdge(const Mat& edgeImage)
  {
    vector<int> edgePoints;
    for(int r = 0; r < edgeImage.rows; r++)
    {
      for(int c = 0; c < edgeImage.cols; c++)
      {
        if(edgeImage.at<uchar>(r, c) > 127)
        {
          edgePoints.push_back(c);
          edgePoints.push_back(r);
        }
      }
    }
    return edgePoints;
  }

  Index<L2<int> > buildIndex(vector<int> features, const IndexParams& params)
  {
    Index<L2<int> > index(Matrix<int>(&features[0], features.size()/2, 2), params);
    index.buildIndex();
    return index;
  }

  float diff(const Index<L2<int> >& index, vector<int> queries, const float radius)
  {
    Matrix<int> q(&queries[0], queries.size()/2, 2);
    Matrix<int> indices(new int[q.rows], q.rows, 1);
    Matrix<float> dists(new float[q.rows], q.rows, 1);
    index.radiusSearch(q, indices, dists, radius, SearchParams());
    float sum = 0;
    for(int i = 0; i < dists.rows; i++)
    {
      float d = dists[i][0];
      if(d > radius * radius)
      {
        sum += radius * radius * 2;
      }
      else
      {
        sum += d;
      }
    }
    return sum / dists.rows;
  }

  ImageProcessor::ImageProcessor(const IndexParams& params, const int loThresh, const int hiThresh)
    : it_(nh_)
    , record_seq_(0)
    , trigger_topic_("record_feature")
    , lo_thresh_(loThresh)
    , hi_thresh_(hiThresh)
  {
    params_ = params;
    service_ = nh_.advertiseService(trigger_topic_, &ImageProcessor::triggerCallback, this);
  }

  void ImageProcessor::listenFeature(const string& imageTopic, const string& jointTopic)
  {
    image_flag_ = joint_flag_ = false;
    img_sub_ = it_.subscribe(imageTopic, 2, &ImageProcessor::featureImageCallback, this);
    joint_sub_ = nh_.subscribe(jointTopic, 2, &ImageProcessor::featureJointCallback, this);
  }

  void ImageProcessor::stopListenFeature()
  {
    img_sub_.shutdown();
    joint_sub_.shutdown();
  }

  bool ImageProcessor::triggerCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
  {
    res.success = img_sub_ != NULL && img_sub_.getTopic().size() > 0;
    if(res.success)
    {
      image_flag_ = true;
      joint_flag_ = true;
      while(image_flag_ || joint_flag_);
      res.message = "Information recorded";
    }
    else
    {
      "Subscribers not initialized";
    }
    return true;
  }

  float ImageProcessor::computeCost(const string& imageTopic, const string& jointTopic)
  {
    broadcast_seq_ = 0;
    error_ = 0;
    image_flag_ = false;
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>(jointTopic, 2);
    img_sub_ = it_.subscribe(imageTopic, 2, &ImageProcessor::featureImageCallback, this);
    std::thread t(&ImageProcessor::publishJointState, this);
    t.join();
    return error_;
  }

  void ImageProcessor::featureImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(image_flag_)
    {
      vector<int> edgePoints = convertImage(msg);
      indices_.push_back(buildIndex(edgePoints, params_));
      std::cout << record_seq_ << std::endl;
      record_seq_++;
      image_flag_ = false;
    }
  }

  void ImageProcessor::featureJointCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    if(joint_flag_)
    {
      joint_states_.push_back(*msg);
      joint_flag_ = false;
    }
  }

  void ImageProcessor::queryImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(image_flag_)
    {
      error_ += diff(indices_[broadcast_seq_], convertImage(msg), 64);
      image_flag_ = false;
    }
    //TODO: Make radius an argument
  }

  void ImageProcessor::publishJointState()
  {
    while(broadcast_seq_ < record_seq_)
    {
      joint_pub_.publish(joint_states_[broadcast_seq_]);
      image_flag_ = true;
      while(image_flag_);
      broadcast_seq_++;
    }
  }

  vector<int> ImageProcessor::convertImage(const sensor_msgs::ImageConstPtr& msg)
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
    Mat edgeImg = detectEdge(cv_ptr->image, lo_thresh_, hi_thresh_);
    return locateEdge(edgeImg);
  }
}

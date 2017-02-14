#include "img_proc.h"

using std::vector;
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

  ImageProcessor::ImageProcessor(const IndexParams& params)
    : it_(nh_)
    , record_flag_(false)
    , lo_thresh_(100)
    , hi_thresh_(160)
  {
    params_ = params;
  }

  void ImageProcessor::recordFeature(char*& imageTopic, char*& jointTopic, const int numRecords,
    const int loThresh, const int hiThresh)
  {
    total_ = numRecords;
    record_seq_ = 0;
    indices_.clear();
    joint_states_.clear();
    lo_thresh_ = loThresh;
    hi_thresh_ = hiThresh;
    img_sub_ = it_.subscribe(imageTopic, 2, &ImageProcessor::featureImageCallback, this);
    joint_sub_ = nh_.subscribe(jointTopic, 2, &ImageProcessor::featureJointCallback, this);
  }

  float ImageProcessor::computeCost(char*& imageTopic, char*& edgeTopic)
  {
    broadcast_seq_ = 0;
    error_ = 0;
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>(edgeTopic, 2);
    img_sub_ = it_.subscribe(imageTopic, 2, &ImageProcessor::featureImageCallback, this);
    publishJointState(); //TODO: Needs a separate thread
    return error_;
  }

  void ImageProcessor::featureImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    record_flag_ = true;
    vector<int> edgePoints = convertImage(msg);
    indices_.push_back(buildIndex(edgePoints, params_));
    std::cout << record_seq_ << std::endl;
    record_seq_++;
    if(record_seq_ >= total_)
    {
      img_sub_.shutdown();
      joint_sub_.shutdown();
    }
  }

  void ImageProcessor::featureJointCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    if(record_flag_)
    {
      record_flag_ = false;
      joint_states_.push_back(*msg);
    }
  }

  void ImageProcessor::queryImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    error_ += diff(indices_[broadcast_seq_], convertImage(msg), 64);
    //TODO: Make radius an argument
  }

  void ImageProcessor::publishJointState()
  {
    while(broadcast_seq_ < total_)
    {
      joint_pub_.publish(joint_states_[broadcast_seq_]);
      //TODO: Wait for img_sub_ to get an image and do the calculation
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

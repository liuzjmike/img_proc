#include "img_proc.h"

using std::vector;
using std::string;
using cv::Mat;
using cv::Size;
using namespace flann;

namespace img_proc
{
ImageProcessor::ImageProcessor(const IndexParams& params, const int loThresh, const int hiThresh)
: it_(nh_)
, record_seq_(0)
, broadcast_seq_(0)
, error_(0)
, image_flag_(false)
, joint_flag_(false)
, cost_flag_(false)
, lo_thresh_(loThresh)
, hi_thresh_(hiThresh)
{
	params_ = params;
	compare_server_ = nh_.advertiseService("compute_cost", &ImageProcessor::costCallback, this);
}

void ImageProcessor::listenFeature(const string& imageTopic, const string& jointTopic)
{
	image_flag_ = joint_flag_ = false;
	record_img_sub_ = it_.subscribe(imageTopic, 2, &ImageProcessor::featureImageCallback, this);
	joint_sub_ = nh_.subscribe(jointTopic, 2, &ImageProcessor::featureJointCallback, this);
	record_server_ = nh_.advertiseService("record_feature", &ImageProcessor::recordCallback, this);
}

Mat ImageProcessor::canny(const Mat& image, const int loThresh, const int hiThresh)
{
	Mat gray, blurImg;
	cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	blur(gray, blurImg, Size(3, 3));
	Canny(blurImg, gray, loThresh, hiThresh);
	return gray;
}

vector<float> ImageProcessor::locateEdge(const Mat& edgeImage)
{
	vector<float> edgePoints;
	for(float r = 0; r < edgeImage.rows; r++)
	{
		for(float c = 0; c < edgeImage.cols; c++)
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

float ImageProcessor::diff(vector<float> features, vector<float> queries)
{
	Index<L2<float> > index(Matrix<float>(&features[0], features.size()/2, 2), params_);
	index.buildIndex();
	Matrix<float> q(&queries[0], queries.size()/2, 2);
	Matrix<int> indices(new int[q.rows], q.rows, 1);
	Matrix<float> dists(new float[q.rows], q.rows, 1);
	index.knnSearch(q, indices, dists, 1, SearchParams());
	float sum = 0;
	for(int i = 0; i < q.rows; i++)
	{
		sum += dists[i][0];
	}
	return sum / dists.rows;
}

bool ImageProcessor::recordCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
	res.success = record_img_sub_ != NULL && record_img_sub_.getTopic().size() > 0;
	if(res.success)
	{
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

bool ImageProcessor::costCallback(img_proc::ComputeCost::Request& req, img_proc::ComputeCost::Response& res)
{
	broadcast_seq_ = 0;
	error_ = 0;
	cost_flag_ = false;
	std::thread t(&ImageProcessor::publishJointState, this, req.joint_topic);
	cost_img_sub_ = it_.subscribe(req.image_topic, 2, &ImageProcessor::queryImageCallback, this);
	t.join();
	res.cost = error_ / broadcast_seq_;
	return true;
}

void ImageProcessor::featureImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if(image_flag_)
	{
		features_.push_back(imageToEdgePos(msg));
		std::cout << record_seq_ << std::endl;
		record_seq_++;
		image_flag_ = false;
	}
}

void ImageProcessor::featureJointCallback(const sensor_msgs::JointStateConstPtr& msg)
{
	if(joint_flag_)
	{
		if(samePosition(position_, msg->position)) {
			joint_states_.push_back(*msg);
			joint_flag_ = false;
			image_flag_ = true;
		}
		position_ = msg->position;
	}
}

void ImageProcessor::queryImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if(cost_flag_)
	{
		error_ += diff(features_[broadcast_seq_], imageToEdgePos(msg));
		cost_flag_ = false;
		broadcast_seq_++;
	}
}

void ImageProcessor::publishJointState(const string& jointTopic)
{
	joint_pub_ = nh_.advertise<sensor_msgs::JointState>(jointTopic, 2);
	while(broadcast_seq_ < record_seq_ - 1)
	{
    std::cout << broadcast_seq_ << std::endl;
    sensor_msgs::JointState state = joint_states_[broadcast_seq_];
    state.header.stamp = ros::Time::now();
    joint_pub_.publish(state);
    cost_flag_ = true;
    while(!cost_flag_);
	}
}

vector<float> ImageProcessor::imageToEdgePos(const sensor_msgs::ImageConstPtr& msg)
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
	Mat edgeImg = canny(cv_ptr->image, lo_thresh_, hi_thresh_);
	return locateEdge(edgeImg);
}

bool ImageProcessor::samePosition(const vector<double>& position1, const vector<double>& position2)
{
	if(position1.size() != position2.size())
	{
		return false;
	}
	else
	{
		for(int i = 0; i < position1.size(); i++)
		{
			if(std::abs(position1[i] - position2[i]) > threshold)
			{
				return false;
			}
		}
		return true;
	}
}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "img_proc");
	img_proc::ImageProcessor ip(flann::KDTreeIndexParams(8));
	ip.listenFeature(argv[1], argv[2]);
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();
	return 0;
}

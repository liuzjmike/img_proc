#include "record_controller.h"

using std::vector;
using std::string;

namespace img_proc
{
RecordController::RecordController(string prefix, vector<string> joints) :
		as_(nh_, action_name, boost::bind(&RecordController::fullRecord, this, _1), false),
		record_seq_(0)
{
	for(int i = 0; i < joints.size(); i++)
	{
		client_ = nh_.serviceClient<std_srvs::Trigger>("record_feature");
		string topic = prefix + joints[i] + topic_suffix;
		joint_pub_.push_back(nh_.advertise<std_msgs::Float64>(topic, 2));
		as_.start();
	}
}

void RecordController::fullRecord(const img_proc::RecordGoalConstPtr &goal)
{
	feedback_.percent_finished = 0;
	record_seq_ = 0;
	if((result_.success = recursiveFullRecord(goal->num_per_joint, 0)))
	{
		as_.setSucceeded(result_);
	}
}

bool RecordController::recursiveFullRecord(int numPerJoint, int index)
{
	ros::Publisher publisher = joint_pub_[index];
	for(int i = 0; i < numPerJoint; i++)
	{
		if (as_.isPreemptRequested() || !ros::ok())
		{
			ROS_INFO("%s: Preempted", action_name.c_str());
			return false;
		}
		std_msgs::Float64 msg;
		msg.data = pi * (2. * i / numPerJoint - 1);
		publisher.publish(msg);
		if(index < joint_pub_.size() - 1)
		{
			if(!recursiveFullRecord(numPerJoint, index + 1))
			{
				return false;
			}
		}
		else
		{
			std_srvs::Trigger srv;
			if(!client_.call(srv))
			{
				ROS_ERROR("Failed to call service record_feature");
				return false;
			}
			else
			{
				record_seq_++;
				feedback_.percent_finished = record_seq_ / pow(numPerJoint, joint_pub_.size()) * 100;
				as_.publishFeedback(feedback_);
			}
		}
	}
	return true;
}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_controller");
	vector<string> joints;
	for(int i = 2; i < argc; i++)
	{
		joints.push_back(argv[i]);
	}
	img_proc::RecordController rc(argv[1], joints);
	ros::spin();
	return 0;
}

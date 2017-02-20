#include "joint_controller.h"

using std::vector;
using std::string;

namespace img_proc
{
  JointController::JointController(string prefix, vector<string> joints)
  {
    for(int i = 0; i < joints.size(); i++)
    {
      client_ = nh_.serviceClient<std_srvs::Trigger>("record_feature");
      string topic = prefix + joints[i] + topic_suffix_;
      joint_pub_.push_back(nh_.advertise<std_msgs::Float64>(topic, 2));
    }
  }

  void JointController::fullRecord(int numPerJoint)
  {
    fullRecord(numPerJoint, 0);
  }

  void JointController::fullRecord(int numPerJoint, int index)
  {
    ros::Publisher publisher = joint_pub_[index];
    for(int i = 0; i < numPerJoint; i++)
    {
      std_msgs::Float64 msg;
      msg.data = pi * (2. * i / numPerJoint - 1);
      publisher.publish(msg);
      if(index < joint_pub_.size() - 1)
      {
        fullRecord(numPerJoint, index + 1);
      }
      else
      {
        std_srvs::Trigger srv;
        if(!client_.call(srv))
        {
          ROS_ERROR("Failed to call service record_feature");
        }
      }
    }
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
  img_proc::JointController jc(argv[1], joints);
  jc.fullRecord();
  ros::spin();
  return 0;
}

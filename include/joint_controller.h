#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Trigger.h"

#include <cmath>

#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

namespace img_proc
{
  class JointController
  {
  public:
    JointController(std::string prefix, std::vector<std::string> joints);

    void fullRecord(int numPerJoint = 30);

  private:
    void fullRecord(int numPerJoint, int index);

    const std::string topic_suffix_ = "_position_controller/command";
    const double pi = 3.14159;
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    std::vector<ros::Publisher> joint_pub_;
  };
}

#endif

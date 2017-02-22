#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <img_proc/RecordAction.h>
#include <img_proc/RecordActionFeedback.h>
#include <img_proc/RecordActionResult.h>
#include "std_msgs/Float64.h"
#include "std_srvs/Trigger.h"

#include <cmath>

#ifndef RECORD_CONTROLLER_H
#define RECORD_CONTROLLER_H

namespace img_proc
{
  class RecordController
  {
  public:
    RecordController(std::string prefix, std::vector<std::string> joints);

  private:
    void fullRecord(const img_proc::RecordGoalConstPtr &goal);
    bool recursiveFullRecord(int numPerJoint, int index);

    const std::string topic_suffix = "_position_controller/command", action_name = "record";
    const double pi = 3.14159;

    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<img_proc::RecordAction> as_;
    img_proc::RecordFeedback feedback_;
    img_proc::RecordResult result_;

    ros::ServiceClient client_;
    std::vector<ros::Publisher> joint_pub_;

    int record_seq_;
  };
}

#endif

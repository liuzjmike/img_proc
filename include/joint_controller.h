#include "ros/ros.h"

#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

namespace img_proc
{
  class JointController
  {
  public:
    JointController();

  private:
    ros::Subscriber joint_sub_;
    ros::Publisher joint_pub_;
  }
}

#endif

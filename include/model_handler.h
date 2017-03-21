/*
 * model_handler.h
 *
 *  Created on: Mar 20, 2017
 *      Author: Mike Liu
 */

#include "ros/ros.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/tree.hpp"
#include "img_proc/JointChange.h"

#include <string>
#include <vector>

#ifndef INCLUDE_MODEL_HANDLER_H_
#define INCLUDE_MODEL_HANDLER_H_

namespace img_proc
{
  class ModelHandler
  {
  public:
    ModelHandler(std::string prefix);
    bool updateModel(std::vector<std::string>& links, std::vector<float> changes);

  private:
    bool updateModel(img_proc::JointChange::Request& req, img_proc::JointChange::Response& res);

    ros::NodeHandle nh_;
    ros::ServiceServer server_;
    std::string model_;
  };
}

#endif /* INCLUDE_MODEL_HANDLER_H_ */

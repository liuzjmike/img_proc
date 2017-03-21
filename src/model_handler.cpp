/*
 * model_handler.cpp
 *
 *  Created on: Mar 20, 2017
 *      Author: Mike Liu
 */

#include "model_handler.h"

using std::string;
using std::vector;

namespace img_proc
{
ModelHandler::ModelHandler(string prefix)
{
  model_ = prefix + "/robot_description";
  server_ = nh_.advertiseService("compute_cost", &ModelHandler::updateModel, this);
}

bool ModelHandler::updateModel(vector<std::string>& links, std::vector<float> changes)
{
  KDL::Tree my_tree;
  std::string robot_desc_string;
  nh_.param(model_, robot_desc_string, string());
  if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }
  return true;
}

bool ModelHandler::updateModel(img_proc::JointChange::Request& req, img_proc::JointChange::Response& res)
{
  res.success = updateModel(req.links, req.changes);
  return res.success;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "model_handler");
  img_proc::ModelHandler mh(argv[1]);
  ros::spin();
  return 0;
}

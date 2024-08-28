/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <ros/ros.h>
#include "box_post_processor/BoxPostProcessor.hpp"

int main(int argc, char* argv[0]) {
  ros::init(argc, argv, "box_post_processor");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  // const std::string paramFolderPath = tryGetParam<std::string>("parameter_folder_path", *nh);
  // const std::string paramFilename = box_post_processor::tryGetParam<std::string>("parameter_filename", *nh);

  // // The LUA parameters are loaded twice. This is the first time. Soley because we need to know if we are using a map for initialization.
  // SlamParameters params;
  // io_lua::loadParameters(paramFolderPath, paramFilename, &params);

  // const bool isProcessAsFastAsPossible = box_post_processor::tryGetParam<bool>("is_read_from_rosbag", *nh);
  // std::cout << "Is process as fast as possible: " << std::boolalpha << isProcessAsFastAsPossible << "\n";
  // std::cout << "Is use a map for initialization: " << std::boolalpha << params.mapper_.isUseInitialMap_ << "\n";
  // std::cout << "Is Map carving enabled: " << std::boolalpha << params.mapper_.isCarvingEnabled_ << "\n";

  // This is where the initial class is constructed and passed on.
  std::shared_ptr<box_post_processor::BoxPostProcessor> hesaiReplayer = std::make_shared<box_post_processor::BoxPostProcessor>(nh);

  hesaiReplayer->initialize();
  // hesaiReplayer->startProcessing();

  ros::shutdown();
  return 0;
}
#include <ros/ros.h>
#include "box_post_processor/box_tf_processor.hpp"

int main(int argc, char* argv[0]) {
  ros::init(argc, argv, "box_tf_processor");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  std::shared_ptr<box_post_processor::BoxTFProcessor> tfReplayer = std::make_shared<box_post_processor::BoxTFProcessor>(nh);

  tfReplayer->initialize();

  ros::shutdown();
  return 0;
}
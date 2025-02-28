#include <ros/ros.h>
#include "box_post_processor/box_imu_processor.hpp"

int main(int argc, char* argv[0]) {
  ros::init(argc, argv, "box_post_processor");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  std::shared_ptr<box_post_processor::BoxPostProcessor> boxReplayer = std::make_shared<box_post_processor::BoxPostProcessor>(nh);

  boxReplayer->initialize();

  ros::shutdown();
  return 0;
}
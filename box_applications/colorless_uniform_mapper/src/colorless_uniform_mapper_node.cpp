#include <ros/ros.h>

#include "colorless_uniform_mapper/ColorlessMapper.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "colorless_uniform_mapper");
  ros::NodeHandle nodeHandle("~");

  colorless_uniform_mapper::ColorlessMapper colorlessMappper(nodeHandle);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}

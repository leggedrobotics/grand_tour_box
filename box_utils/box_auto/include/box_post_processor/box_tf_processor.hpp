#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace box_post_processor {

class BoxTFProcessor {
 public:
  BoxTFProcessor(ros::NodeHandlePtr nh);
  ~BoxTFProcessor() = default;

  void initialize();
  inline double elapsedMilliseconds() { return std::chrono::duration_cast<std::chrono::milliseconds>(endTime_ - startTime_).count(); }
  inline double elapsedSeconds() { return elapsedMilliseconds() / 1000.0; }

 private:
  std::string rosbagFullname_;
  std::string outputBagFolder_;
  std::chrono::time_point<std::chrono::steady_clock> startTime_;
  std::chrono::time_point<std::chrono::steady_clock> endTime_;

  // std::string buildUpLogFilename(const std::string& typeSuffix, const std::string& extension = ".txt");
  bool createOutputDirectory();
  bool processRosbags(std::vector<std::string>& tfContainingBags);

  rosbag::Bag outBag;
  // create ros handle
  ros::NodeHandlePtr nh_;

  uint64_t counter_ = 0;
};

}  // namespace box_post_processor

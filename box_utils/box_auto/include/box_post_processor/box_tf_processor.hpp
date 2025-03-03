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

struct RotationOverride {
  std::string parent_frame;
  std::string child_frame;
  std::string axis;
  double angle_deg;
};

struct BoxTfParams {
  std::vector<std::pair<std::string, std::string>> frameMapping;
  std::vector<std::pair<std::string, std::string>> framePairs;
  std::vector<std::string> childFramesToRemove;
  std::vector<std::string> parentFramesToRemove;
  std::vector<std::string> childFramesToInverse;
  std::vector<RotationOverride> rotationOverrides;
};

class BoxTFProcessor {
 public:
  BoxTFProcessor(ros::NodeHandlePtr nh);
  ~BoxTFProcessor() = default;

  bool combineTransforms(tf2_ros::Buffer& tfBuffer, std::vector<tf2_msgs::TFMessage>& tfStatic, const std::string& startFrame,
                         const std::string& targetFrame);

  void updateFrameNames(std::vector<tf2_msgs::TFMessage>& tfMsgs, const std::vector<std::pair<std::string, std::string>>& frameMapping);
  std::string extractDatePrefix(const std::string& globalPath);
  std::string createOutputBagName(const std::string& inputGlobalPath, const std::string& outputSuffix);
  bool loadBoxTfParameters(ros::NodeHandle& nh, BoxTfParams& params);
  void applyRotationOverrides(std::vector<tf2_msgs::TFMessage>& tfStaticMsgs);

  void initialize();
  inline double elapsedMilliseconds() { return std::chrono::duration_cast<std::chrono::milliseconds>(endTime_ - startTime_).count(); }
  inline double elapsedSeconds() { return elapsedMilliseconds() / 1000.0; }

 private:
  std::string rosbagFullname_;
  std::string outputBagFolder_;
  std::chrono::time_point<std::chrono::steady_clock> startTime_;
  std::chrono::time_point<std::chrono::steady_clock> endTime_;

  bool createOutputDirectory();
  bool processRosbags(std::vector<std::string>& tfContainingBags);

  rosbag::Bag outBag;
  // create ros handle
  ros::NodeHandlePtr nh_;

  uint64_t counter_ = 0;
  std::vector<std::string> frames_;
  BoxTfParams params;
  double tfStaticRepetitionPeriod_ = 5;
};

}  // namespace box_post_processor

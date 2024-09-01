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

class BoxPostProcessor {
 public:
  BoxPostProcessor(ros::NodeHandlePtr nh);
  ~BoxPostProcessor() = default;

  void initialize();
  inline double elapsedMilliseconds() { return std::chrono::duration_cast<std::chrono::milliseconds>(endTime_ - startTime_).count(); }
  inline double elapsedSeconds() { return elapsedMilliseconds() / 1000.0; }

 private:
  std::string rosbagFullname_;
  std::string rosbagFilename_;
  std::string outputBagFolder_;
  std::chrono::time_point<std::chrono::steady_clock> startTime_;
  std::chrono::time_point<std::chrono::steady_clock> endTime_;

  std::ofstream poseFile_;
  std::ofstream imuFile_;
  ros::Time baseTime_ = ros::Time(0);

  // std::string buildUpLogFilename(const std::string& typeSuffix, const std::string& extension = ".txt");
  bool createOutputDirectory();
  bool processRosbag();
  void createZedBasedTransform(tf2_msgs::TFMessage& collectiontfMessage);

  //! Tf2.
  tf2_ros::TransformBroadcaster transformBroadcaster_;
  tf2_ros::StaticTransformBroadcaster staticTransformBroadcaster_;

  //! Tf topic name.
  std::string tfTopic_{"/tf"};

  //! Static Tf topic name.
  std::string tfStaticTopic_{"/tf_static"};

  bool isFirstMessage_ = true;

  ros::Duration timeDiff_;
  rosbag::Bag outBag;

  // create ros handle
  ros::NodeHandlePtr nh_;

  geometry_msgs::TransformStamped baseToLidarTransform_;
  std::string odometryHeader_{"/bestHeaderThereis"};
  std::vector<geometry_msgs::TransformStamped> staticTransforms_;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;

  bool isBagReadyToPlay_ = false;
  uint64_t counter_ = 0;
  bool foundTfmsgs_ = false;
};

}  // namespace box_post_processor

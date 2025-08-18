// MIT License

// Copyright (c) 2023-2025 ETH Zurich, Robotic Systems Lab.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
  std::string outputRosbagFilename_;
  std::string inputRosbagFilename_;
  std::chrono::time_point<std::chrono::steady_clock> startTime_;
  std::chrono::time_point<std::chrono::steady_clock> endTime_;

  std::ofstream poseFile_;
  std::ofstream imuFile_;
  ros::Time baseTime_ = ros::Time(0);

  ros::Time oldTime_ = ros::Time(0);

  uint32_t oldSeq_ = 300;
  uint32_t seq_ = 0;

  // std::string buildUpLogFilename(const std::string& typeSuffix, const std::string& extension = ".txt");
  // bool createOutputDirectory();
  bool processRosbag();
  // void createZedBasedTransform(tf2_msgs::TFMessage& collectiontfMessage);

  //! Tf2.
  tf2_ros::TransformBroadcaster transformBroadcaster_;
  tf2_ros::StaticTransformBroadcaster staticTransformBroadcaster_;

  std::vector<std::string> topics_;

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
  double maxTimeDifference_ = 0.0;
  double time_offset_to_be_applied_ = 0.0;
  std::map<std::string, double> timeOffsetsToApply_;
  std::map<std::string, double> maxTimeDifferences_;
  std::map<std::string, std::vector<std::string>> allTopics_;
  std::string mode_;
};

}  // namespace box_post_processor

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

struct RotationOverride {
  std::string parent_frame;
  std::string child_frame;
  std::string axis;
  double angle_deg;
};

struct BoxTfParams {
  std::vector<std::tuple<std::string, std::string, bool>> frameMapping;
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

  void updateFrameNames(std::vector<tf2_msgs::TFMessage>& tfMsgs,
                        const std::vector<std::tuple<std::string, std::string, bool>>& frameMapping);
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

/*
 * BoxPostProcessor.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <deque>
#include <memory>
#include <string>
#include <vector>

// //#include <hesai_lidar_sdk.hpp>
// #include "lidar.h"
// #include "lidar_types.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include "open3d_slam/SlamWrapper.hpp"
// #include "open3d_slam/output.hpp"

#include <std_srvs/Empty.h>

// #include <hesai_ros_driver/UdpFrame.h>
// #include <hesai_ros_driver/UdpPacket.h>
// // #include "open3d_slam_ros/DataProcessorRos.hpp"

namespace box_post_processor {

// struct hesaiSlamInputs {
//   sensor_msgs::PointCloud2::ConstPtr pointCloud_;
//   geometry_msgs::PoseStamped::ConstPtr odometryPose_;

//   hesaiSlamInputs() = default;

//   hesaiSlamInputs(sensor_msgs::PointCloud2::ConstPtr pointCloud, geometry_msgs::PoseStamped::ConstPtr odometryPose)
//       : pointCloud_(pointCloud), odometryPose_(odometryPose) {}
// };

class BoxPostProcessor {
  // using BASE = DataProcessorRos;
  // using SlamInputsBuffer = std::deque<std::unique_ptr<hesaiSlamInputs>>;

 public:
  BoxPostProcessor(ros::NodeHandlePtr nh);
  ~BoxPostProcessor() = default;

  void initialize();
  // void startProcessing() override;
  // void processMeasurement(const PointCloud& cloud, const Time& timestamp) override;

  // void poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  // /**
  //  * @brief Processes a buffer of SLAM inputs stored in memory.
  //  *
  //  * @param buffer  Buffer of SLAM inputs, FIFO.
  //  */
  // bool processBuffers(SlamInputsBuffer& buffer);

  // void drawLinesBetweenPoses(const nav_msgs::Path& path1, const nav_msgs::Path& path2, const ros::Time& stamp);

  // std::optional<visualization_msgs::Marker> generateMarkersForSurfaceNormalVectors(const open3d::geometry::PointCloud& o3d_pc,
  //                                                                                  const ros::Time& timestamp,
  //                                                                                  const box_post_processor::RgbaColorMap::Values&
  //                                                                                  color);

  // /**
  //  * @brief Validates that essential topics required to run sequential evaluation are present in the bag file.
  //  *
  //  * @param bag               ROS bag file to analyze.
  //  * @param mandatoryTopics   List of mandatory topics.
  //  * @return true             If all mandatory topics are present in the data, false otherwise.
  //  */
  // bool validateTopicsInRosbag(const rosbag::Bag& bag, const std::vector<std::string>& mandatoryTopics);

  // bool readCalibrationIfNeeded();

  // /*!
  //  * @brief Run sequential SLAM.
  //  * @return true If successful, false otherwise.
  //  */
  // bool run();

  // std::tuple<ros::WallDuration, ros::WallDuration, ros::WallDuration> usePairForRegistration();

  inline double elapsedMilliseconds() { return std::chrono::duration_cast<std::chrono::milliseconds>(endTime_ - startTime_).count(); }

  inline double elapsedSeconds() { return elapsedMilliseconds() / 1000.0; }

 private:
  // void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  // bool processRosbag();
  // sensor_msgs::PointCloud2 ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id);
  std::string rosbagFullname_;
  std::string rosbagFilename_;
  std::string outputBagFolder_;

  //! Parameters.
  // Parameters parameters_;

  std::chrono::time_point<std::chrono::steady_clock> startTime_;
  std::chrono::time_point<std::chrono::steady_clock> endTime_;

  nav_msgs::Path trackedPath_;
  nav_msgs::Path bestGuessPath_;
  std::ofstream poseFile_;
  std::ofstream imuFile_;
  std::string asyncOdometryFrame_;
  ros::Time baseTime_ = ros::Time(0);

  // std::string buildUpLogFilename(const std::string& typeSuffix, const std::string& extension = ".txt");
  bool createOutputDirectory();
  bool processRosbag();
  // visualization_msgs::MarkerArray convertPathToMarkerArray(const nav_msgs::Path& path);
  // visualization_msgs::Marker createLineStripMarker();

  // box_post_processor::PointCloud lineStripToPointCloud(const visualization_msgs::MarkerArray& marker_array, const int num_samples);

  // void calculateSurfaceNormals(box_post_processor::PointCloud& cloud);

  // void processRosbagForIMU();

  // void exportIMUData();

  //! Publishers.
  ros::Publisher clockPublisher_;
  ros::Publisher inputPointCloudPublisher_;

  ros::Publisher odometryPosePublisher_;
  ros::Publisher registeredPosePublisher_;

  ros::ServiceServer sleepServer_;

  sensor_msgs::PointCloud2 registeredCloud_;

  //! Tf2.
  tf2_ros::TransformBroadcaster transformBroadcaster_;
  tf2_ros::StaticTransformBroadcaster staticTransformBroadcaster_;

  std::deque<geometry_msgs::PoseStamped> registeredPoses_;

  ros::Time tracker;

  //! Maximum processing rate.
  double maxProcessingRate_{-0.1};

  //! Tf topic name.
  std::string tfTopic_{"/tf"};

  //! Static Tf topic name.
  std::string tfStaticTopic_{"/tf_static"};

  //! ROS clock topic name.
  std::string clockTopic_{"/clock"};

  bool isFirstMessage_ = true;
  bool isStaticTransformFound_ = false;

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

  // box_post_processor::RgbaColorMap colorMap_;
};

}  // namespace box_post_processor

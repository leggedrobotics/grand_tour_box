#pragma once

// standard lib.
#include <algorithm>
#include <cmath>
#include <mutex>

// Ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

// geometry_msgs
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Maintained Map types
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/transform_listener.h>

// Simple ring buffer
#include "colorless_uniform_mapper/circularRingBuffer.hpp"

namespace colorless_uniform_mapper {

struct MapperParameters {
  //! ID of the coordinate frame to accumulate point clouds in.
  std::string mappingFrame_{"my_map_your_world"};  // map or camera_init

  //! Name of the input point cloud topic.
  std::string inputPointCloudTopic_{"/point_cloud_filter/lidar/point_cloud_filtered"};

  //! Name of the output topic
  std::string localOutputPointCloudTopicName_{"/colorless_uniform_mapper/local_point_cloud"};

  //! Position based request topic.
  std::string positionBasedRequestTopic_{"/clicked_point"};

  std::string positionBasedAlternativeRequestTopic_{"/glimpse_interactive_marker/requested_mesh_point"};

  //! To identify different maps from different robots.
  std::string robotIdentifer_{""};

  //! Wait time for Tf lookup time.
  double tfLookUpTime_{0.01};

  //! Local fetching radius. [m]
  double radius_{8.0};

  //! Point cloud buffering period
  double bufferingPeriod_{10.0};

  //! Voxel size for uniform mapping.
  double octreeVoxelSize_{0.03f};

  //! Map saving period.
  double mapSavingPeriod_{180};

  //! Map saving size ratio.
  int mapSavingSizeDifference_{500000};

  //! Map saving folder path.
  std::string mapSavePath_{""};

  //! Time in seconds for automatic map publishing.
  double localMapPublishingPeriod_{10.0};

  //! Height clipping. Currently expects Z axis to be the height representative axis.
  double heightClipping_{2.0};

  //! Transportation Voxel size. Should be larger (sparser) than mapping voxel size.
  float transportationVoxelSize_{0.075f};

  //! Min neighbourhood for point filtering.
  int minNeighbourhood_{5};

  //! The filtering radius to get rid of the noise.
  float noiseFilteringRadius_{0.1f};

  //! The circular ring buffer size.
  int ringBufferSize_{5};

  //! The threshold to stop publishing the same cloud over and over again.
  int publishPointThreshold_{2000};
};

class ColorlessMapper {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  explicit ColorlessMapper(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ColorlessMapper() = default;

  //! Read mapper parameters from ROS Parameter Server.
  bool readMapperParameters();

  /**
   * @brief Set the Up Ros Transport objects (publishers, services, subscribers and timers)
   *
   */
  void setUpRosTransport();

  // For colorless.
  using PointType = pcl::PointXYZI;
  using PointCloud = pcl::PointCloud<PointType>;
  using Octree = pcl::octree::OctreePointCloudSearch<PointType>;

 private:
  //! Initialize the mapper.
  void initializeMappper();

  // Check saving path.
  bool manageSaveDirectory();

  // Create path recursively.
  bool makePath(const std::string& path);

  // Check whether directory exits
  bool doesDirectoryExist(const std::string& path);

  //! Fetch the points around the robot.
  void getLocalPoints(const pcl::PointXYZI searchPoint);

  //! Processes the received point cloud.
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

  //! Save the map callback. Relays message to the function.
  bool saveMapToFileCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);

  //! Save the map locally.
  bool saveMapToFile(const std::string& mapName, const bool isAutomatic);

  //! Publish the map based on a timer.
  void publishMapTimerCallback(const ros::TimerEvent&);

  //! Local map publishing callback.
  void localMapPublishingTimerCallback(const ros::TimerEvent&);

  //! Map saving timer callback
  void mapSavingTimerCallback(const ros::TimerEvent&);

  //! Send the local point cloud within a given radius.
  bool sendLocalCloudCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);

  //! Reset the colorless mapper.
  bool resetCb(std_srvs::EmptyRequest& /*request*/, std_srvs::EmptyResponse& /*response*/);

  //! Transform point cloud to the mapping frame.
  bool transformCloudToAccumulationFrame(const std::string& fixedFrame, const std::string& targetFrame, const ros::Time& targetStamp,
                                         const std::string& sourceFrame, const ros::Time& sourceStamp, PointCloud::Ptr pointCloud);

  //! Add points from the buffer.
  void addPoints(const PointCloud::Ptr pointCloud);

  //! Add points from the buffer.
  void positionBasedRequestCallback(const geometry_msgs::PointStampedConstPtr& pointStamped);

  //! Add value to the circular map buffer.
  std::optional<std::string> addMapToRingBuffer(std::string mapName) { return autosavedMapRingBuffer_.addValue(mapName); }

  //! Initialize the map buffer with a fixed size.
  void initializeRingBuffer();

  //! Delete file from filesystem.
  bool deleteFile(const std::string& filePath);

  //! ros wall time as string
  std::string timeToString(const ros::WallTime& stamp, const std::string format);

  //! Make a mess in the terminal
  void printPoint(PointType& point);

  // A ring buffer for the autosaved maps.
  CircularRingBuffer autosavedMapRingBuffer_;

  //! Parameters.
  MapperParameters parameters_;

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Publishers
  ros::Publisher localMapPublisher_;
  ros::Publisher inlierPublisher_;
  ros::Publisher previousPub;

  //! Point cloud subscriber.
  ros::Subscriber pointCloudSubscriber_;
  ros::Subscriber positionBasedRequestSubscriber_;
  ros::Subscriber positionBasedAlternativeRequestSubscriber_;

  //! Save point cloud to file server.
  ros::ServiceServer saveMapToFileServer_;
  ros::ServiceServer localPointCloudSenderServer_;
  ros::ServiceServer resetServer_;

  //! Point cloud publishing timer.
  ros::Timer mapPublishingTimer_;
  ros::Timer mapSavingTimer_;
  ros::Timer localMapPublishingTimer_;

  //! Mutexes for multi threding.
  mutable std::mutex bufferOfPointCloudsMutex_;
  mutable std::mutex mapMutex_;
  mutable std::mutex poseVectorMutex_;

  //! Buffer of incoming point clouds.
  PointCloud::Ptr bufferOfPointClouds_{new PointCloud()};
  //! Accumulated point cloud.
  PointCloud::Ptr mapPointCloud_{new PointCloud()};
  //! Map octree.
  Octree::Ptr octreeMap_{new Octree(parameters_.octreeVoxelSize_)};

  //! TF buffers
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  //! Pose of lidar in the tracked frame.
  geometry_msgs::TransformStamped sensorPoseInMappingFrame_;

  //! The point where we ask for the local patch around.
  PointType searchPoint_;

  //! The map size holder.
  std::size_t mapLeafSize_{0u};

  //! Whether the extraction process is in place.
  bool isExtractionIsInProcess_{false};

  //! Manual map save counter
  int manualSaveCounter_{1};

  //! Previously filtered Local patchvoid ColorlessMapper::cropBoxPoseSetCallback_1(const geometry_msgs::PoseStampedConstPtr& pose)
  PointCloud::Ptr previousPatch_{new PointCloud()};
};

}  // namespace colorless_uniform_mapper

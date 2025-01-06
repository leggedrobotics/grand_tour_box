
#include "colorless_uniform_mapper/ColorlessMapper.hpp"

// C++ standard library
#include <errno.h>
#include <ros/package.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <filesystem>
#include <iostream>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

namespace colorless_uniform_mapper {

ColorlessMapper::ColorlessMapper(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle), tfListener_(tfBuffer_) {
  if (!readMapperParameters()) {
    ROS_ERROR("Error reading parameters for colorless mapper class. Program will exit.");
    return;
  }

  // Initialize the colorless mapper.
  initializeMappper();

  // Initialize the ros transport.
  setUpRosTransport();

  // Set the ring buffer size.
  initializeRingBuffer();

  // Prematurely check correctness and existance of save path.
  if (!manageSaveDirectory()) {
    return;
  }

  ROS_INFO("Initialized node for dense colorless mapping.");
}

bool ColorlessMapper::manageSaveDirectory() {
  if (parameters_.mapSavePath_.empty()) {
    // Get username
    std::string userName{getlogin()};
    ROS_INFO_STREAM("User name: " << userName);
    parameters_.mapSavePath_ = "/home/" + userName + "/maps";

    if (!doesDirectoryExist(parameters_.mapSavePath_)) {
      if (makePath(parameters_.mapSavePath_)) {
        ROS_INFO_STREAM("Created folder : " << parameters_.mapSavePath_);
      } else {
        ROS_ERROR_STREAM("Could not create folder " << parameters_.mapSavePath_ << " Exitting program.");
        return false;
      }
    }

    ROS_INFO_STREAM("Outputting results (as requested) to: (" << parameters_.mapSavePath_ << ")");

  } else {
    // Check if folder exists
    if (!doesDirectoryExist(parameters_.mapSavePath_)) {
      // If does not exist create the folder recursively.
      if (makePath(parameters_.mapSavePath_)) {
        ROS_INFO_STREAM("Created folder : " << parameters_.mapSavePath_);
      } else {
        // Quit if the folder was not created.
        ROS_ERROR_STREAM("Could not create folder " << parameters_.mapSavePath_ << "Exitting program.");
        return false;
      }
    }

    ROS_INFO_STREAM("Outputting results (as requested) to: " << parameters_.mapSavePath_);
  }

  return true;
}

bool ColorlessMapper::makePath(const std::string& path) {
  // C magic, read, write and execute rights.
  const mode_t mode{0777};

  if (mkdir(path.c_str(), mode) == 0) {
    return true;
  }

  // if the deepest folder was not possible to create check parents.
  switch (errno) {
    case ENOENT:
      // Parent didn't exist, try to create it
      {
        std::size_t lastSlashPosition{path.find_last_of('/')};
        if (lastSlashPosition == std::string::npos) {
          // If the slash is at the end, nothing to do.
          return false;
        }
        // Get the path until the previous `/` and create that path.
        if (!makePath(path.substr(0, lastSlashPosition))) {
          return false;
        }
      }
      // now, try to create again
      return 0 == mkdir(path.c_str(), mode);

    case EEXIST:
      // done!
      return doesDirectoryExist(path);

    default:
      return false;
  }
}

bool ColorlessMapper::doesDirectoryExist(const std::string& path) {
  struct stat info;

  if (stat(path.c_str(), &info) != 0) {
    return false;
  }

  return (info.st_mode & S_IFDIR) != 0;
}

void ColorlessMapper::initializeRingBuffer() {
  autosavedMapRingBuffer_ = CircularRingBuffer(static_cast<size_t>(parameters_.ringBufferSize_));
}

void ColorlessMapper::initializeMappper() {
  // Clear existing structures.
  if (!mapPointCloud_->empty()) {
    mapPointCloud_->clear();
  }
  if (octreeMap_) {
    octreeMap_->deleteTree();
  }

  // Init data structures.
  mapPointCloud_.reset(new PointCloud());

  // Initialize octree map empty.
  octreeMap_.reset(new Octree(parameters_.octreeVoxelSize_));
  octreeMap_->setInputCloud(mapPointCloud_);
}

bool ColorlessMapper::readMapperParameters() {
  ROS_DEBUG_STREAM("Reading the colorless mapper parameters.");
  bool success{true};

  // Read the parameters
  success &= nodeHandle_.param<std::string>("mapping_frame", parameters_.mappingFrame_, "map");
  success &= nodeHandle_.param<std::string>("local_output_point_cloud_topic_name", parameters_.localOutputPointCloudTopicName_,
                                            "/colorless_uniform_mapper/local_point_cloud");
  success &= nodeHandle_.param<std::string>("input_point_cloud_topic", parameters_.inputPointCloudTopic_,
                                            "/point_cloud_filter/lidar_depth_camera/point_cloud_filtered");
  success &= nodeHandle_.param<double>("map_rendering_period", parameters_.bufferingPeriod_, 10.0);
  success &= nodeHandle_.param<double>("tf_lookup_time", parameters_.tfLookUpTime_, 0.01);
  success &= nodeHandle_.param<double>("radius", parameters_.radius_, 8.0);
  success &= nodeHandle_.param<double>("voxel_size", parameters_.octreeVoxelSize_, 0.03);
  success &= nodeHandle_.param<double>("map_saving_period", parameters_.mapSavingPeriod_, 120);
  success &= nodeHandle_.param<int>("map_saving_size_difference", parameters_.mapSavingSizeDifference_, 500000);
  success &= nodeHandle_.param<std::string>("map_saving_path", parameters_.mapSavePath_, "");
  success &= nodeHandle_.param<double>("local_map_publishing_period", parameters_.localMapPublishingPeriod_, 10.0);
  success &= nodeHandle_.param<std::string>("position_based_request_topic", parameters_.positionBasedRequestTopic_, "/clicked_point");
  success &= nodeHandle_.param<std::string>("position_based_alternative_request_topic", parameters_.positionBasedAlternativeRequestTopic_,
                                            "/glimpse_interactive_marker/requested_mesh_point");
  success &= nodeHandle_.param<float>("transportation_voxel_size", parameters_.transportationVoxelSize_, 0.075f);
  success &= nodeHandle_.param<int>("outlier_filter_min_neighbours", parameters_.minNeighbourhood_, 5);
  success &= nodeHandle_.param<float>("noise_filtering_radius", parameters_.noiseFilteringRadius_, 0.1f);
  success &= nodeHandle_.param<int>("ring_buffer_size", parameters_.ringBufferSize_, 5);

  return success;
}

void ColorlessMapper::setUpRosTransport() {
  // Published per request.
  localMapPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(parameters_.localOutputPointCloudTopicName_, 1, false);

  // Publish
  inlierPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("difference_point_cloud", 1, false);
  previousPub = nodeHandle_.advertise<sensor_msgs::PointCloud2>("previous_point_cloud", 1, false);
  pointCloudSubscriber_ = nodeHandle_.subscribe(parameters_.inputPointCloudTopic_, 100, &ColorlessMapper::pointCloudCallback, this);
  positionBasedRequestSubscriber_ =
      nodeHandle_.subscribe(parameters_.positionBasedRequestTopic_, 1, &ColorlessMapper::positionBasedRequestCallback, this);
  positionBasedAlternativeRequestSubscriber_ =
      nodeHandle_.subscribe(parameters_.positionBasedAlternativeRequestTopic_, 1, &ColorlessMapper::positionBasedRequestCallback, this);

  ROS_DEBUG_STREAM("Subscribed to point cloud topic: \"" << pointCloudSubscriber_.getTopic() << "\".");

  // Service servers.
  saveMapToFileServer_ = nodeHandle_.advertiseService("save_pointcloud_map", &ColorlessMapper::saveMapToFileCallback, this);
  localPointCloudSenderServer_ = nodeHandle_.advertiseService("send_local_point_cloud", &ColorlessMapper::sendLocalCloudCallback, this);
  resetServer_ = nodeHandle_.advertiseService("reset_colorless_mapper", &ColorlessMapper::resetCb, this);

  // Timers.
  mapPublishingTimer_ =
      nodeHandle_.createTimer(ros::Duration(parameters_.bufferingPeriod_), &ColorlessMapper::publishMapTimerCallback, this);

  mapSavingTimer_ = nodeHandle_.createTimer(ros::Duration(parameters_.mapSavingPeriod_), &ColorlessMapper::mapSavingTimerCallback, this);

  localMapPublishingTimer_ = nodeHandle_.createTimer(ros::Duration(parameters_.localMapPublishingPeriod_),
                                                     &ColorlessMapper::localMapPublishingTimerCallback, this);
}

bool ColorlessMapper::resetCb(std_srvs::EmptyRequest& /*request*/, std_srvs::EmptyResponse& /*response*/) {
  ROS_INFO("Colorless mapper is resetting.");

  {
    std::lock_guard<std::mutex> methodLock(mapMutex_);
    initializeMappper();
  }

  {
    std::lock_guard<std::mutex> lock(bufferOfPointCloudsMutex_);
    if (!bufferOfPointClouds_->empty()) {
      bufferOfPointClouds_->clear();
    }

    bufferOfPointClouds_.reset(new PointCloud());
  }

  ROS_INFO("Reset is successful.");
  return true;
}

void ColorlessMapper::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc_cloud) {
  const std::size_t pcSize = pc_cloud->height * pc_cloud->width;
  ROS_DEBUG_STREAM("Received point cloud with " << pcSize << " points.");
  if (pcSize < 100) {
    ROS_WARN_STREAM("The received point cloud is very small with size: " << pcSize);
    return;
  }

  // Deserialize point cloud msg from ros type to pcl type.
  PointCloud::Ptr pclPointCloud(new PointCloud());
  pcl::fromROSMsg(*pc_cloud, *pclPointCloud);

  // Fetch the pose of the lidar in the mapping frame.
  if (!transformCloudToAccumulationFrame(parameters_.mappingFrame_, parameters_.mappingFrame_, pc_cloud->header.stamp,
                                         pc_cloud->header.frame_id, pc_cloud->header.stamp, pclPointCloud)) {
    ROS_WARN("Point cloud could not be transformed to mapping frame. Skipping.");
    return;
  }

  {
    // Add point cloud to buffer.
    std::lock_guard<std::mutex> lock(bufferOfPointCloudsMutex_);
    *bufferOfPointClouds_ += *pclPointCloud;
  }
}

void ColorlessMapper::localMapPublishingTimerCallback(const ros::TimerEvent&) {
  ROS_DEBUG_STREAM("Automatic map publishing timer is triggered. Sending local PC.");

  if (isExtractionIsInProcess_) {
    return;
  }
  getLocalPoints(searchPoint_);
}

void ColorlessMapper::mapSavingTimerCallback(const ros::TimerEvent&) {
  const std::size_t localSize = octreeMap_->getLeafCount();
  const std::size_t diffInLeafs = localSize - mapLeafSize_;

  ROS_DEBUG_STREAM("Automatic saving timer difference: " << diffInLeafs << " threshold: " << parameters_.mapSavingSizeDifference_);

  if (diffInLeafs > parameters_.mapSavingSizeDifference_) {
    ROS_INFO_STREAM("Dense map will be automatically saved. New map size: " << localSize);

    // Saved per request.
    const std::string mapName = "colorless_map_auto";
    bool success = saveMapToFile(mapName, true);

    if (success) {
      mapLeafSize_ = localSize;

    } else {
      ROS_ERROR("Couldnt_automatically_save");
    }
  } else {
    ROS_DEBUG("Difference in map is not big enough to trigger a save.");
  }
}

std::string ColorlessMapper::timeToString(const ros::WallTime& stamp, const std::string format) {
  std::array<char, 100> stringTime{};
  const std::time_t rawTime{static_cast<time_t>(stamp.sec)};
  struct tm* timeInfo = localtime(&rawTime);
  std::strftime(reinterpret_cast<char*>(stringTime.data()), 100, format.c_str(), timeInfo);
  std::stringstream ss;
  ss << std::setw(9) << std::setfill('0') << stamp.nsec;
  return std::string(reinterpret_cast<char const*>(stringTime.data())) + "_" + ss.str().substr(0, 4);
}

void ColorlessMapper::publishMapTimerCallback(const ros::TimerEvent&) {
  ROS_DEBUG("Adding poiints to tree.");

  const std::size_t bufferSize = bufferOfPointClouds_->points.size();

  if (bufferSize < 100) {
    ROS_WARN_STREAM("Tried to merge point cloud buffer. Buffer is very sparce with NbPoints: " << bufferSize << ". Skipping merging.");
    return;
  }

  // Lock the map
  std::lock_guard<std::mutex> methodLock(mapMutex_);
  {
    // Lock the buffer
    std::lock_guard<std::mutex> bufferLock(bufferOfPointCloudsMutex_);

    // Add buffer to map.
    addPoints(bufferOfPointClouds_);
    bufferOfPointClouds_->clear();
  }
}

void ColorlessMapper::addPoints(const PointCloud::Ptr points) {
  for (size_t i{0u}; i < points->points.size(); ++i) {
    const PointType& point{points->points[i]};
    if (!octreeMap_->isVoxelOccupiedAtPoint(point)) {
      octreeMap_->addPointToCloud(point, mapPointCloud_);
    }
  }
}

bool ColorlessMapper::saveMapToFileCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  ROS_DEBUG("Manual saving of point cloud to file service callback.");

  // Saved per request.
  const std::string mapName = std::to_string(manualSaveCounter_) + "_colorless_map_manual";
  bool success = saveMapToFile(mapName, false);
  manualSaveCounter_++;
  return true;
}

bool ColorlessMapper::sendLocalCloudCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  // Fetch and send local point cloud.
  if (isExtractionIsInProcess_) {
    return false;
  }

  getLocalPoints(searchPoint_);
  return true;
}

void ColorlessMapper::positionBasedRequestCallback(const geometry_msgs::PointStampedConstPtr& pointStamped) {
  if (pointStamped->header.frame_id != parameters_.mappingFrame_) {
    ROS_WARN_STREAM("Inconsistent frame " << pointStamped->header.frame_id << " to request local patch. Returning.");
    return;
  }

  // Fetch the neighbourhood near the requested point.
  PointType searchPoint;
  searchPoint.x = static_cast<float>(pointStamped->point.x);
  searchPoint.y = static_cast<float>(pointStamped->point.y);
  searchPoint.z = static_cast<float>(pointStamped->point.z);

  ROS_DEBUG_STREAM("Position based request is received, for position: x:" << searchPoint.x << " y: " << searchPoint.y
                                                                          << " z: " << searchPoint.z);
  getLocalPoints(searchPoint);
}

void ColorlessMapper::getLocalPoints(const pcl::PointXYZI searchPoint) {
  ROS_DEBUG("Publishing local point cloud.");
  const std::size_t sizeOfMap = mapPointCloud_->points.size();

  if (sizeOfMap < 100) {
    ROS_WARN_STREAM("Map is empty with size:  '" << sizeOfMap << ". Skipping publishing local point cloud.");
    return;
  }

  // Set operational status.
  isExtractionIsInProcess_ = true;

  // Initialize containers
  PointCloud::Ptr sub_cloud(new PointCloud);
  std::vector<int> pointIdxBoxSearch;

  // Point in mapping frame. (maybe explicitly set to pcl::XYZ)
  std::vector<float> pointRadiusSquaredDistance;

  ROS_DEBUG("Starting radius search");

  try {
    std::lock_guard<std::mutex> methodLock(mapMutex_);
    octreeMap_->radiusSearch(searchPoint, parameters_.radius_, pointIdxBoxSearch, pointRadiusSquaredDistance);

  } catch (const std::exception& e) {
    ROS_ERROR(e.what());
    return;
  } catch (...) {
    ROS_ERROR("Caught an unknown exception \n");
    return;
  }

  ROS_DEBUG_STREAM("Size of extracted indices '" << pointIdxBoxSearch.size());

  // Get the point cloud from a given vector of indices.
  pcl::ExtractIndices<PointType> extract;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  inliers->indices = pointIdxBoxSearch;
  std::lock_guard<std::mutex> lock(mapMutex_);
  { extract.setInputCloud(mapPointCloud_); }
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*sub_cloud);

  if (sub_cloud->points.size() < 100) {
    ROS_WARN("Local point cloud is empty returning.");
    return;
  }

  ROS_DEBUG_STREAM("Voxeling down for transportation");

  // Downsampling filter. For efficient transportation. Expects the voxel size in 3 dimmensions.
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud(sub_cloud);
  sor.setLeafSize(parameters_.transportationVoxelSize_, parameters_.transportationVoxelSize_, parameters_.transportationVoxelSize_);
  sor.filter(*sub_cloud);

  ROS_DEBUG_STREAM("Removing outliers.");

  // // Point cloud filtering. Remove points that are outliers. For a given radius, filters if there are not enough neighbourhing points
  // pcl::RadiusOutlierRemoval<PointType> rorfilter;
  // rorfilter.setInputCloud(sub_cloud);
  // rorfilter.setRadiusSearch(parameters_.noiseFilteringRadius_);
  // rorfilter.setMinNeighborsInRadius(parameters_.minNeighbourhood_);
  // rorfilter.setNegative(false);
  // rorfilter.filter(*sub_cloud);

  if (sub_cloud->points.size() < 100) {
    ROS_WARN("After outlier filtering local cloud is empty, returning.");
    return;
  }

  if (localMapPublisher_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr localPC{boost::make_shared<sensor_msgs::PointCloud2>()};
    pcl::toROSMsg(*sub_cloud, *localPC);
    localPC->header.frame_id = parameters_.mappingFrame_;
    localPC->header.stamp = ros::Time::now();
    localMapPublisher_.publish(localPC);
  }

  // // Check whether there is a change in the local patch (idea in backlog since we can't stop sending data `assuming` we received it.
  // // This redundancy is important.)
  // float resolution = parameters_.octreeVoxelSize_;
  // // Instantiate octree-based point cloud change detection class
  // pcl::octree::OctreePointCloudChangeDetector<PointType> changeTree(resolution);

  // changeTree.setInputCloud(previousPatch_);
  // changeTree.addPointsFromInputCloud();
  // changeTree.switchBuffers();

  // // Add points from cloudB to octree
  // changeTree.setInputCloud(sub_cloud);
  // changeTree.addPointsFromInputCloud();

  // std::vector<int> newPointIdxVector;

  // // Get vector of point indices from octree voxels which did not exist in previous buffer
  // changeTree.getPointIndicesFromNewVoxels(newPointIdxVector);

  // if (inlierPublisher_.getNumSubscribers() > 0) {
  //   PointCloud::Ptr inlierCloud(new PointCloud);
  //   pcl::ExtractIndices<PointType> changedPointExtractor;
  //   pcl::PointIndices::Ptr changedPointIndices(new pcl::PointIndices());
  //   changedPointIndices->indices = newPointIdxVector;

  //   changedPointExtractor.setInputCloud(sub_cloud);
  //   changedPointExtractor.setIndices(changedPointIndices);
  //   changedPointExtractor.setNegative(false);
  //   changedPointExtractor.filter(*inlierCloud);

  //   sensor_msgs::PointCloud2::Ptr InlierPointCloudMsg{boost::make_shared<sensor_msgs::PointCloud2>()};
  //   pcl::toROSMsg(*inlierCloud, *InlierPointCloudMsg);

  //   // Since mapping was in a given frame, the points fetched from it are also in the same frame.
  //   InlierPointCloudMsg->header.frame_id = parameters_.mappingFrame_;
  //   InlierPointCloudMsg->header.stamp = ros::Time::now();
  //   inlierPublisher_.publish(InlierPointCloudMsg);
  // }

  // if (previousPub.getNumSubscribers() > 0) {
  //   sensor_msgs::PointCloud2::Ptr previousMsgs{boost::make_shared<sensor_msgs::PointCloud2>()};
  //   pcl::toROSMsg(*previousPatch_, *previousMsgs);

  //   // Since mapping was in a given frame, the points fetched from it are also in the same frame.
  //   previousMsgs->header.frame_id = parameters_.mappingFrame_;
  //   previousMsgs->header.stamp = ros::Time::now();
  //   previousPub.publish(previousMsgs);
  // }

  // previousPatch_ = sub_cloud;

  // Operations are done.
  isExtractionIsInProcess_ = false;
}

void ColorlessMapper::printPoint(PointType& point) {
  std::ostringstream pointSS;
  pointSS << std::fixed << std::setprecision(4);
  pointSS << "[ " << point.x << ",  " << point.y << ",  " << point.z << " ] ";
  std::string pointString = pointSS.str();
  std::cout << pointString << std::endl;
}

bool ColorlessMapper::saveMapToFile(const std::string& mapName, const bool isAutomatic) {
  // Save the map to the file.
  std::string fileName = "";

  // Generate the file name itself.
  std::string timeAsString = timeToString(ros::WallTime::now(), "%Y_%m_%d_%H_%M_%S");
  fileName = parameters_.mapSavePath_ + "/" + timeAsString + "_" + mapName + ".ply";

  ROS_INFO_STREAM("Requested save full path:  " << fileName);

  try {
    std::lock_guard<std::mutex> lock(mapMutex_);
    pcl::PLYWriter writer;
    if (writer.write(fileName, *mapPointCloud_, true, false) != 0) {
      ROS_ERROR("Something went wrong when trying to write the point cloud file.");
      return false;
    }
    ROS_DEBUG("Map point cloud has been saved successfully.");
    if (isAutomatic) {
      const auto& fileToRemove{addMapToRingBuffer(fileName)};
      if (fileToRemove.has_value()) {
        if (!deleteFile(fileToRemove.value())) {
          ROS_WARN("Cannot delete %s", fileToRemove.value().c_str());
        }
        ROS_WARN("Deleted old map '%s'.", fileToRemove.value().c_str());
      }
    }
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR(e.what());
    return false;
  } catch (...) {
    ROS_ERROR("Caught an unknown exception \n");
    return false;
  }

  // Shouldnt be able to come to this point actually..
  return true;
}

bool ColorlessMapper::deleteFile(const std::string& filePath) {
  if (std::filesystem::exists(filePath)) {
    return std::filesystem::remove(filePath);
  }

  ROS_WARN_STREAM("The autosaved file '" << filePath << "' does not exist. The map autosave file might be already deleted.");
  return false;
}

bool ColorlessMapper::transformCloudToAccumulationFrame(const std::string& fixedFrame, const std::string& targetFrame,
                                                        const ros::Time& targetStamp, const std::string& sourceFrame,
                                                        const ros::Time& sourceStamp, PointCloud::Ptr pointCloud) {
  if (sourceFrame == targetFrame && sourceStamp == targetStamp) {
    ROS_DEBUG("Early return same frames.");
    return true;
  }

  if (!tfBuffer_.canTransform(targetFrame, targetStamp, sourceFrame, sourceStamp, fixedFrame, ros::Duration(parameters_.tfLookUpTime_))) {
    ROS_WARN_STREAM("Requested transform from frames \'" + sourceFrame + "' to '" + targetFrame + "', with target timestamp "
                    << targetStamp << " cannot be found.");
    return false;
  }

  geometry_msgs::TransformStamped tfSourceToTargetRos;
  try {
    // Lookup tf from source frame to target frame, using an intermediate fixed frame
    tfSourceToTargetRos =
        tfBuffer_.lookupTransform(targetFrame, targetStamp, sourceFrame, sourceStamp, fixedFrame, ros::Duration(parameters_.tfLookUpTime_));
  } catch (const tf2::TransformException& exception) {
    ROS_DEBUG_STREAM("Caught an exception while looking up transformation: " << exception.what());
    return false;
  }

  // Transform point cloud from source frame to target frame.
  pcl_ros::transformPointCloud(*pointCloud, *pointCloud, tfSourceToTargetRos.transform);

  searchPoint_.x = static_cast<float>(tfSourceToTargetRos.transform.translation.x);
  searchPoint_.y = static_cast<float>(tfSourceToTargetRos.transform.translation.y);
  searchPoint_.z = static_cast<float>(tfSourceToTargetRos.transform.translation.z);

  return true;
}

}  // namespace colorless_uniform_mapper

/*
 * BoxPostProcessor.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "box_post_processor/BoxPostProcessor.hpp"
#include <open3d/io/PointCloudIO.h>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <filesystem>

namespace box_post_processor {

BoxPostProcessor::BoxPostProcessor(ros::NodeHandlePtr nh) : nh_(nh) {}

void BoxPostProcessor::initialize() {
  rosbagFilename_ = nh_->param<std::string>("rosbag_filepath", "");
  outputBagFolder_ = nh_->param<std::string>("map_saving_folder", "");
  ROS_INFO_STREAM("Reading from rosbag: " << rosbagFilename_);

  createOutputDirectory();
  rosbagFullname_ = outputBagFolder_ + "/testtest.bag";
  std::remove(rosbagFullname_.c_str());

  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(2.0)};
  ros::WallTime::sleepUntil(first);

  processRosbag();

  ros::shutdown();
}

bool BoxPostProcessor::createOutputDirectory() {
  // Check if the output folder exists.
  if (!std::filesystem::is_directory(outputBagFolder_)) {
    // If the folder doesn't exist, create it.
    try {
      return std::filesystem::create_directories(outputBagFolder_);
    } catch (const std::exception& exception) {
      ROS_ERROR_STREAM("Caught an exception trying to create output folder: " << exception.what());
    }
  }

  return false;
}

bool BoxPostProcessor::processRosbag() {
  std::vector<std::string> topics;

  topics.push_back("/tf");

  // Create me a high resolution clock timer from std chrono
  // Start the timer.
  startTime_ = std::chrono::steady_clock::now();

  // Open ROS bag.
  rosbag::Bag bag;
  try {
    bag.open(rosbagFilename_, rosbag::bagmode::Read);
  } catch (const rosbag::BagIOException& e) {
    ROS_ERROR_STREAM("Error opening ROS bag: '" << rosbagFilename_ << "'");
    return false;
  }

  outBag.open(rosbagFullname_, rosbag::bagmode::Write);
  // ROS_INFO_STREAM("ROS bag '" << rosbagFilename_ << "' open.");
  // if (!validateTopicsInRosbag(bag, topics)) {
  //   bag.close();
  //   return false;
  // }

  ROS_INFO_STREAM("\033[92m"
                  << " Post processing the Rosbag "
                  << "\033[0m");
  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(1.0)};
  ros::WallTime::sleepUntil(first);

  // // The bag view we iterate over.
  rosbag::View view1(bag, rosbag::TopicQuery(topics));

  // ros::Time stampLastIteration{view.getBeginTime()};
  // ros::WallTime wallStampLastIteration{ros::WallTime::now()};
  // const ros::WallTime wallStampStartSequentialRun{ros::WallTime::now()};

  std::vector<ros::Time> timeStamps;
  for (const auto& messageInstance : view1) {
    // If the node is shutdown, stop processing and do early return.
    if (!ros::ok()) {
      return false;
    }

    if (messageInstance.getTopic() == "/tf") {
      // ROS_INFO_STREAM("NORMAL TF FOUND");
      tf2_msgs::TFMessage::ConstPtr message = messageInstance.instantiate<tf2_msgs::TFMessage>();
      if (message != nullptr) {
        if (baseTime_.toSec() == 0.0) {
          baseTime_ = message->transforms[0].header.stamp;
        }

        tf2_msgs::TFMessage collectionTFMessage_ = *message;

        timeStamps.push_back(message->transforms[0].header.stamp);

        outBag.write("/tf", message->transforms[0].header.stamp, collectionTFMessage_);
      }
    }
  }

  std::vector<std::string> topics2;
  topics2.push_back("/tf_static");
  rosbag::View view2(bag, rosbag::TopicQuery(topics2));

  tf2_msgs::TFMessage collectiontfMessage_;

  for (const auto& messageInstance : view2) {
    // If the node is shutdown, stop processing and do early return.
    if (!ros::ok()) {
      return false;
    }

    bool isInvalidMessageInBag = false;

    // Update time registers.
    // if ((ros::Time::now() - stampLastIteration) >= ros::Duration(1.0)) {
    //   stampLastIteration = ros::Time::now();
    //   wallStampLastIteration = ros::WallTime::now();
    // }
    // ROS_INFO_STREAM("Starting bag");

    if (messageInstance.getTopic() == "/tf_static") {
      ROS_INFO_STREAM("TF STATIC FOUND");
      tf2_msgs::TFMessage::ConstPtr message = messageInstance.instantiate<tf2_msgs::TFMessage>();
      if (message != nullptr) {
        // ROS_INFO_STREAM("Stamp: " << message->transforms[0].header.stamp);
        // std::vector<geometry_msgs::TransformStamped> newTransforms;

        // Iterate over the transforms and if the frame_id is not the same as the child_frame_id, then we have a static transform.
        for (auto& transform : message->transforms) {
          if (!(transform.header.frame_id == "prism" || transform.child_frame_id == "prism")) {
            if (!(transform.header.frame_id == "leica_world" || transform.child_frame_id == "leica_world")) {
              if (!(transform.header.frame_id == "leica_base" || transform.child_frame_id == "leica_base")) {
                if (!(transform.header.frame_id == "leica_pos" || transform.child_frame_id == "leica_pos")) {
                  if (!(transform.header.frame_id == "leica_base_model" || transform.child_frame_id == "leica_base_model")) {
                    geometry_msgs::TransformStamped newTransformStamped = transform;
                    // newTransformStamped.header.stamp = baseTime_;

                    // // NODELET_DEBUG_STREAM(get_logger(), "Odom TS: " << transformStamped.header.stamp);

                    // transformStamped.header.frame_id = mOdomFrameId;
                    // transformStamped.child_frame_id = mBaseFrameId;
                    // // conversion from Tranform to message
                    // mOdomMutex.lock();  //
                    // tf2::Vector3 translation = mOdom2BaseTransf.getOrigin();
                    // tf2::Quaternion quat = mOdom2BaseTransf.getRotation();
                    // mOdomMutex.unlock();  //
                    // transformStamped.transform.translation.x = translation.x();
                    // transformStamped.transform.translation.y = translation.y();
                    // transformStamped.transform.translation.z = translation.z();
                    // transformStamped.transform.rotation.x = quat.x();
                    // transformStamped.transform.rotation.y = quat.y();
                    // transformStamped.transform.rotation.z = quat.z();
                    // transformStamped.transform.rotation.w = quat.w();

                    // transform.header.stamp = baseTime_;
                    collectiontfMessage_.transforms.push_back(newTransformStamped);
                  }
                }
              }
            }

            // staticTransformBroadcaster_.sendTransform(transform);
          }
        }
        geometry_msgs::TransformStamped boxToBoxBase;
        boxToBoxBase.header.frame_id = "base";
        boxToBoxBase.header.stamp = baseTime_;
        boxToBoxBase.child_frame_id = "box_base";
        // conversion from Tranform to message

        tf2::Vector3 translation = tf2::Vector3(-0.0361, 0.2178, 0.07640);

        // X Y Z W
        tf2::Quaternion quat = tf2::Quaternion(0.5, -0.5, 0.5, -0.5);

        boxToBoxBase.transform.translation.x = translation.x();
        boxToBoxBase.transform.translation.y = translation.y();
        boxToBoxBase.transform.translation.z = translation.z();
        boxToBoxBase.transform.rotation.x = quat.x();
        boxToBoxBase.transform.rotation.y = quat.y();
        boxToBoxBase.transform.rotation.z = quat.z();
        boxToBoxBase.transform.rotation.w = quat.w();
        collectiontfMessage_.transforms.push_back(boxToBoxBase);

        // ROS_WARN_STREAM("Header stamp: " << baseTime_);
        // outBag.write("/tf_static", baseTime_, collectiontfMessage_);

      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

  }  // end of for loop

  for (auto& time : timeStamps) {
    for (auto& transform : collectiontfMessage_.transforms) {
      transform.header.stamp = time;
    }

    outBag.write("/tf_static", time, collectiontfMessage_);
  }

  ROS_INFO("Finished running through the bag for IMU msgs.");
  // const ros::Time bag_begin_time = view.getBeginTime();
  // const ros::Time bag_end_time = view.getEndTime();
  endTime_ = std::chrono::steady_clock::now();

  // std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec() << " sec."
  //           << " Time elapsed for processing: " << elapsedSeconds() << " sec. \n \n";

  bag.close();
  outBag.close();
  return true;
}

// std::string BoxPostProcessor::buildUpLogFilename(const std::string& typeSuffix, const std::string& extension) {
//   ros::WallTime stamp = ros::WallTime::now();
//   std::stringstream ss;
//   ss << stamp.sec << "_" << stamp.nsec;

//   // Add prefixes.
//   // Not sure if adding time is the best thing to do since we dont keep a ring buffer.
//   // std::string filename = slam_->mapSavingFolderPath_ + typeSuffix + ss.str() + extension;
//   std::string filename = slam_->mapSavingFolderPath_ + typeSuffix + extension;

//   return filename;
// }

// void BoxPostProcessor::startProcessing() {
//   if (!createOutputDirectory()) {
//     std::cout << "Couldn't create the directory "
//               << "\n";
//     return;
//   }

//   if (slam_->exportIMUdata_) {
//     exportIMUData();
//     std::cout << "IMU Exporting is complete"
//               << "\n";

//     const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(10.0)};
//     ros::WallTime::sleepUntil(first);

//     std::cout << "Sleeping 10 seconds.."
//               << "\n";
//   }

//   std::string trackedPosesFilename_ = buildUpLogFilename("slam_poses");
//   std::remove(trackedPosesFilename_.c_str());

//   const std::string poseLogFileHeader_ = "# timestamp x y z q_x q_y q_z q_w";

//   // Open file and set numerical precision to the max.
//   poseFile_.open(trackedPosesFilename_, std::ios_base::app);
//   poseFile_.precision(std::numeric_limits<double>::max_digits10);
//   poseFile_ << poseLogFileHeader_ << std::endl;

//   std::string outBagPath_ = buildUpLogFilename("processed_slam", ".bag");
//   std::remove(outBagPath_.c_str());
//   outBag.open(outBagPath_, rosbag::bagmode::Write);

//   // Iterate and process the bag.
//   if (processRosbag()) {
//     // Create the magical tube.
//     visualization_msgs::MarkerArray lineStrip = convertPathToMarkerArray(trackedPath_);

//     box_post_processor::PointCloud samplecloud;
//     // Magic number, points per circle strip. If you set it too high, cloud becomes mega heavy.
//     int numPoints = 50;
//     // Define the radius of the tube
//     float tube_radius = 0.02;

//     samplecloud = lineStripToPointCloud(lineStrip, numPoints);

//     calculateSurfaceNormals(samplecloud);

//     // Create a new point cloud for the tube
//     box_post_processor::PointCloud tube_cloud;
//     std::vector<Eigen::Vector3d> tube_cloud_normal_vector;
//     std::vector<Eigen::Vector3d> tube_cloud_point_vector;

//     tube_cloud_normal_vector.reserve(samplecloud.points_.size() * 36);
//     tube_cloud_point_vector.reserve(samplecloud.points_.size() * 36);

//     // Iterate over the points in the original point cloud
//     for (int i = 0; i < samplecloud.points_.size() - numPoints; i++) {
//       // Retrieve the surface normal
//       Eigen::Vector3d normal = samplecloud.normals_[i].normalized();

//       // Calculate the direction vector
//       const Eigen::Vector3d direction = normal.unitOrthogonal().cross(normal).normalized();

//       // Generate points around the current point
//       const Eigen::Vector3d current_point = samplecloud.points_[i];
//       const Eigen::Vector3d current_normal = samplecloud.normals_[i];
//       for (float angle = 0; angle <= 360; angle += 10) {
//         float x = current_point.x() + tube_radius * (direction.x() * cos(angle) + normal.x() * sin(angle));
//         float y = current_point.y() + tube_radius * (direction.y() * cos(angle) + normal.y() * sin(angle));
//         float z = current_point.z() + tube_radius * (direction.z() * cos(angle) + normal.z() * sin(angle));

//         Eigen::Vector3d tube_point(x, y, z);

//         tube_cloud_normal_vector.push_back(current_normal);
//         tube_cloud_point_vector.push_back(tube_point);
//       }
//     }
//     tube_cloud.normals_ = tube_cloud_normal_vector;
//     tube_cloud.points_ = tube_cloud_point_vector;

//     // Copy the surface normals to the new point cloud
//     for (int i = 0; i < samplecloud.points_.size() - numPoints; i++) {
//       for (int j = 0; j < 36; j++) {
//         tube_cloud.normals_[i * 36 + j] = samplecloud.normals_[i];
//       }
//     }

//     std::string nameWithCorrectSuffix = slam_->mapSavingFolderPath_ + "robotPathAsMesh.pcd";
//     // size_t found = nameWithCorrectSuffix.find(".pcd");

//     open3d::io::WritePointCloudToPCD(nameWithCorrectSuffix, tube_cloud, open3d::io::WritePointCloudOption());
//     ROS_INFO_STREAM("Successfully saved the poses as point cloud. Waiting for user to terminate.");
//   }

//   // Close file handle.
//   poseFile_.close();
//   outBag.close();

//   ros::spin();
//   slam_->stopWorkers();
//   return;
// }

// bool BoxPostProcessor::validateTopicsInRosbag(const rosbag::Bag& bag, const std::vector<std::string>& mandatoryTopics) {
//   // Get a view on the data and check if all mandatory topics are present.
//   bool areMandatoryTopicsInRosbag{true};

//   // Iterate over the mandatory topics and check if they are in the bag.
//   for (const auto& topic : mandatoryTopics) {
//     rosbag::View topicView(bag, rosbag::TopicQuery(topic));
//     if (topicView.size() == 0u) {
//       if (topic == clockTopic_) {
//         // This means this is our second time coming here. So actually the the alternative topic is not working either.
//         if (topic == slam_->asyncOdometryTopic_) {
//           ROS_ERROR_STREAM(clockTopic_ << " topic does not exist in the rosbag. This is breaking.");
//           areMandatoryTopicsInRosbag = false;
//         } else {
//           ROS_ERROR_STREAM(clockTopic_ << " topic does not exist in the rosbag. Using alternative topic: " << slam_->asyncOdometryTopic_
//                                        << " as clock.");
//           clockTopic_ = slam_->asyncOdometryTopic_;
//         }

//       } else if (topic == tfTopic_) {
//         ROS_WARN_STREAM("No data under the topic: " << topic << " was found. This was optional so okay.");
//         continue;
//       } else if (topic == tfStaticTopic_) {
//         ROS_ERROR_STREAM("No data under the topic: "
//                          << topic << " was found. This is NOT optional. But if you make tf_static available external its okay.");
//         continue;
//       } else {
//         ROS_ERROR_STREAM("No data under the topic: " << topic << " was found.");
//         areMandatoryTopicsInRosbag = false;
//       }
//     } else {
//       if (topic == slam_->asyncOdometryTopic_) {
//         // Get a view for the specific topic only
//         rosbag::View view(bag, rosbag::TopicQuery(topic));
//         for (const auto& messageInstance : view) {
//           if (messageInstance.getDataType() == "geometry_msgs/PoseWithCovarianceStamped") {
//             ROS_WARN_STREAM(" ' " << topic
//                                   << "' topic does not support automatic frame detection. Msg Type: " << messageInstance.getDataType()
//                                   << ". Assumed tracked odometry frame: " << slam_->frames_.assumed_external_odometry_tracked_frame);
//             break;
//           } else if (messageInstance.getDataType() == "nav_msgs/Odometry") {
//             nav_msgs::Odometry::ConstPtr message = messageInstance.instantiate<nav_msgs::Odometry>();
//             if (message != nullptr) {
//               slam_->frames_.assumed_external_odometry_tracked_frame = message->child_frame_id;
//               ROS_WARN_STREAM(topic << " frame_id is set to: " << slam_->frames_.assumed_external_odometry_tracked_frame);
//               break;
//             }  // if
//           } else {
//             ROS_ERROR_STREAM(topic << " msg type is NOT SUPPORTED");
//             return false;
//           }

//         }  // for
//       }    // if
//     }      // if
//   }        // for

//   if (slam_->useSyncedPoses_) {
//     ROS_WARN_STREAM("Sync poses are enabled. This does not support automatic frame detection. Assumed tracked odometry frame: "
//                     << slam_->frames_.assumed_external_odometry_tracked_frame);
//   }

//   if (!areMandatoryTopicsInRosbag) {
//     ROS_ERROR("All required topics are not within the rosbag. Replay operations are terminated. Waiting user to terminate.");
//     return false;
//   }

//   return true;
// }

// bool BoxPostProcessor::processBuffers(SlamInputsBuffer& buffer) {
//   // This is as fast as the thread can go
//   if (buffer.empty()) {
//     ROS_DEBUG("Empty buffer");
//     return false;
//   }

//   if (slam_->useSyncedPoses_) {
//     // Sync poses are currently is only X-ICP type. Thus not well supported.
//     // Provide the pose prior
//     auto& odometryPose = buffer.front()->odometryPose_;
//     geometry_msgs::Pose odomPose = odometryPose->pose;

//     if (isFirstMessage_) {
//       Eigen::Isometry3d eigenTransform = Eigen::Isometry3d::Identity();
//       slam_->setExternalOdometryFrameToCloudFrameCalibration(eigenTransform);
//     }

//     if (isFirstMessage_ && isStaticTransformFound_) {
//       geometry_msgs::Pose initialPose;
//       initialPose.position = odomPose.position;
//       initialPose.orientation.w = 1.0;
//       initialPose.orientation.z = 0.0;
//       initialPose.orientation.y = 0.0;
//       initialPose.orientation.x = 0.0;
//       slam_->setInitialTransform(box_post_processor::getTransform(initialPose).matrix());
//       isFirstMessage_ = false;
//     }

//     // Add to the odometry buffer.
//     if (!(slam_->addOdometryPoseToBuffer(box_post_processor::getTransform(odomPose), fromRos(odometryPose->header.stamp)))) {
//       std::cout << "Couldn't Add pose to buffer" << std::endl;
//       return false;
//     }

//   } else {
//     // Buffer is empty, means the async odometry poses did not arrive before the point cloud.
//     if (slam_->isOdometryPoseBufferEmpty()) {
//       std::cout << "Odometry Buffer is empty!" << std::endl;
//       return false;
//     }

//     if (!slam_->isInitialTransformSet()) {
//       std::cout << "Initial transform not set yet! Popping the measurement." << std::endl;
//       return false;
//     }
//   }

//   auto& pointCloud = buffer.front()->pointCloud_;

//   // Convert to o3d cloud
//   open3d::geometry::PointCloud cloud;

//   if (!open3d_conversions::rosToOpen3d(pointCloud, cloud, false, true)) {
//     std::cout << "Couldn't convert the point cloud" << std::endl;
//     return false;
//   }

//   const Time timestamp = fromRos(pointCloud->header.stamp);

//   if (!slam_->doesOdometrybufferHasMeasurement(timestamp)) {
//     std::cout << "RosbagReplayer:: Pointcloud is here, pose buffer is not empty but odometry with the right stamp not available yet. "
//                  "Popping the measurement."
//               << std::endl;
//     buffer.pop_front();
//     return false;
//   }

//   // Add the cloud to queue, internally checks if there is a matching odometry pose in the odometryBuffer
//   if (slam_->addRangeScan(cloud, timestamp)) {
//     // std::cout << "Adding cloud with stamp: " << box_post_processor::toString(timestamp) << std::endl;
//     // Timer mapperOnlyTimer_;
//     // mapperOnlyTimer_.startStopwatch();
//     auto timeTuple = usePairForRegistration();
//     // const double timeElapsed = mapperOnlyTimer_.elapsedMsecSinceStopwatchStart();
//   } else {
//     std::cout << "RosbagReplayer:: Couldn't add range scan. Popping this measurement from the buffer." << std::endl;
//     buffer.pop_front();
//     return false;
//   }

//   // Publish the tfs
//   slam_->offlineTfWorker();

//   // Publish Submap markers
//   slam_->offlineVisualizationWorker();

//   // Check the latest registered cloud buffer
//   std::tuple<PointCloud, Time, Transform> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
//   Transform calculatedTransform = std::get<2>(cloudTimePair);

//   std::tuple<Time, Transform> bestGuessTimePair = slam_->getLatestRegistrationBestGuess();
//   Transform bestGuessTransform = std::get<1>(bestGuessTimePair);

//   geometry_msgs::PoseStamped bestGuessPoseStamped;
//   Eigen::Quaterniond bestGuessRotation(bestGuessTransform.rotation());

//   bestGuessPoseStamped.header.stamp = toRos(std::get<0>(bestGuessTimePair));
//   bestGuessPoseStamped.pose.position.x = bestGuessTransform.translation().x();
//   bestGuessPoseStamped.pose.position.y = bestGuessTransform.translation().y();
//   bestGuessPoseStamped.pose.position.z = bestGuessTransform.translation().z();
//   bestGuessPoseStamped.pose.orientation.w = bestGuessRotation.w();
//   bestGuessPoseStamped.pose.orientation.x = bestGuessRotation.x();
//   bestGuessPoseStamped.pose.orientation.y = bestGuessRotation.y();
//   bestGuessPoseStamped.pose.orientation.z = bestGuessRotation.z();
//   bestGuessPath_.poses.push_back(bestGuessPoseStamped);
//   bestGuessPath_.header.stamp = toRos(std::get<0>(bestGuessTimePair));  // This guess supposed to be associated with the pose we give to
//   it. bestGuessPath_.header.frame_id = slam_->frames_.mapFrame;

//   if (offlineBestGuessPathPub_.getNumSubscribers() > 0u || offlineBestGuessPathPub_.isLatched()) {
//     offlineBestGuessPathPub_.publish(bestGuessPath_);
//   }

//   geometry_msgs::PoseStamped poseStamped;
//   Eigen::Quaterniond rotation(calculatedTransform.rotation());

//   poseStamped.header.stamp = toRos(std::get<1>(cloudTimePair));
//   poseStamped.pose.position.x = calculatedTransform.translation().x();
//   poseStamped.pose.position.y = calculatedTransform.translation().y();
//   poseStamped.pose.position.z = calculatedTransform.translation().z();
//   poseStamped.pose.orientation.w = rotation.w();
//   poseStamped.pose.orientation.x = rotation.x();
//   poseStamped.pose.orientation.y = rotation.y();
//   poseStamped.pose.orientation.z = rotation.z();
//   trackedPath_.poses.push_back(poseStamped);

//   outBag.write("/slam_optimized_poses", toRos(std::get<1>(cloudTimePair)), poseStamped);

//   const double stamp = pointCloud->header.stamp.toSec();
//   poseFile_ << stamp << " ";
//   poseFile_ << calculatedTransform.translation().x() << " " << calculatedTransform.translation().y() << " "
//             << calculatedTransform.translation().z() << " ";
//   poseFile_ << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << std::endl;

//   trackedPath_.header.stamp = toRos(std::get<1>(cloudTimePair));
//   trackedPath_.header.frame_id = slam_->frames_.mapFrame;

//   if (offlinePathPub_.getNumSubscribers() > 0u || offlinePathPub_.isLatched()) {
//     offlinePathPub_.publish(trackedPath_);
//   }

//   drawLinesBetweenPoses(trackedPath_, bestGuessPath_, toRos(std::get<1>(cloudTimePair)));

//   if (isTimeValid(std::get<1>(cloudTimePair)) && !(std::get<0>(cloudTimePair).IsEmpty())) {
//     box_post_processor::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos(std::get<1>(cloudTimePair)),
//                            registeredCloudPub_);
//     ros::spinOnce();
//   } else {
//     ROS_ERROR_STREAM("Cloud is empty or time is invalid. Skipping the cloud.");
//     return false;
//   }

//   if (surfaceNormalPub_.getNumSubscribers() > 0u || surfaceNormalPub_.isLatched()) {
//     auto surfaceNormalLineMarker{
//         generateMarkersForSurfaceNormalVectors(std::get<0>(cloudTimePair), toRos(std::get<1>(cloudTimePair)),
//         colorMap_[ColorKey::kRed])};

//     if (surfaceNormalLineMarker != std::nullopt) {
//       // ROS_DEBUG("Publishing point cloud surface normals for publisher '%s'.", parameters_.pointCloudPublisherTopic_.c_str());
//       surfaceNormalPub_.publish(surfaceNormalLineMarker.value());
//     }
//   }

//   // Pop oldest input point cloud from the queue.
//   buffer.pop_front();

//   sensor_msgs::PointCloud2 outCloud;
//   open3d_conversions::open3dToRos(std::get<0>(cloudTimePair), outCloud, slam_->frames_.rangeSensorFrame);
//   outCloud.header.stamp = toRos(std::get<1>(cloudTimePair));
//   outBag.write("/registered_cloud", toRos(std::get<1>(cloudTimePair)), outCloud);

//   // Convert geometry_msgs::PoseStamped to geometry_msgs::TransformStamped
//   geometry_msgs::TransformStamped transformStamped;
//   transformStamped.header.stamp = poseStamped.header.stamp;
//   transformStamped.header.frame_id = slam_->frames_.mapFrame;
//   transformStamped.child_frame_id = slam_->frames_.rangeSensorFrame;  // The child frame_id should be your robot's frame
//   transformStamped.transform.translation.x = poseStamped.pose.position.x;
//   transformStamped.transform.translation.y = poseStamped.pose.position.y;
//   transformStamped.transform.translation.z = poseStamped.pose.position.z;
//   transformStamped.transform.rotation = poseStamped.pose.orientation;

//   // Encapsulate geometry_msgs::TransformStamped into tf2_msgs/TFMessage
//   tf2_msgs::TFMessage tfMessage;
//   tfMessage.transforms.push_back(transformStamped);

//   // Write the TFMessage to the bag
//   outBag.write("/tf", toRos(std::get<1>(cloudTimePair)), tfMessage);

//   PointCloud& transformedCloud = std::get<0>(cloudTimePair);

//   transformedCloud.Transform(std::get<2>(cloudTimePair).matrix());
//   sensor_msgs::PointCloud2 transformedRosCloud;
//   open3d_conversions::open3dToRos(transformedCloud, transformedRosCloud, slam_->frames_.rangeSensorFrame);
//   transformedRosCloud.header.stamp = toRos(std::get<1>(cloudTimePair));
//   outBag.write("/transformed_registered_cloud", toRos(std::get<1>(cloudTimePair)), transformedRosCloud);

//   const ros::WallTime arbitrarySleep{ros::WallTime::now() + ros::WallDuration(slam_->relativeSleepDuration_)};
//   ros::WallTime::sleepUntil(arbitrarySleep);

//   return true;
// }

}  // namespace box_post_processor

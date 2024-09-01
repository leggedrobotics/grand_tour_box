#include "box_post_processor/BoxPostProcessor.hpp"
#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <filesystem>

namespace box_post_processor {

BoxPostProcessor::BoxPostProcessor(ros::NodeHandlePtr nh) : nh_(nh) {}

void BoxPostProcessor::initialize() {
  inputFolder_ = nh_->param<std::string>("input_folder", "");

  namespace fs = std::filesystem;
  for (const auto& entry : fs::directory_iterator(inputFolder_)) {
    if (entry.path().extension() == ".bag" && entry.path().filename().string().find("_zed") != std::string::npos) {
      rosbagFilename_ = entry.path().string();
      break;
    }
  }
  if (rosbagFilename_.empty()) {
    ROS_ERROR("No matching *_zed.bag file found in the input folder.");
  }

  outputBagFolder_ = nh_->param<std::string>("map_saving_folder", "");
  ROS_INFO_STREAM("Reading from rosbag: " << rosbagFilename_);

  rosbagFullname_ = rosbagFilename_;

  rosbagFullname_.erase(rosbagFullname_.end() - 4, rosbagFullname_.end());

  rosbagFullname_ += "_post_processed.bag";
  ROS_INFO_STREAM("Writing to rosbag: " << rosbagFullname_);

  // createOutputDirectory();
  // rosbagFullname_ = outputBagFolder_ + "/testtest.bag";
  if (std::filesystem::exists(rosbagFullname_)) {
    std::remove(rosbagFullname_.c_str());
  }

  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(2.0)};
  ros::WallTime::sleepUntil(first);

  processRosbag();

  ROS_INFO_STREAM("\033[92m"
                  << " SUCCESSFULLY COMPLETED REPLAYING. TERMINATING MYSELF. "
                  << "\033[0m");

  // ros::shutdown();
  return;
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
      foundTfmsgs_ = true;
      // ROS_INFO_STREAM("NORMAL TF FOUND");
      tf2_msgs::TFMessage::ConstPtr message = messageInstance.instantiate<tf2_msgs::TFMessage>();
      if (message != nullptr) {
        if (baseTime_.toSec() == 0.0) {
          baseTime_ = message->transforms[0].header.stamp;
        }

        tf2_msgs::TFMessage collectionTFMessage_ = *message;

        timeStamps.push_back(message->transforms[0].header.stamp);

        // outBag.write("/tf", message->transforms[0].header.stamp, collectionTFMessage_);
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
      // ROS_INFO_STREAM("TF STATIC FOUND");
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
                    if (transform.child_frame_id == "zed_base") {
                      newTransformStamped.child_frame_id = "zed_base_link";
                    }

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
        boxToBoxBase.header.stamp = collectiontfMessage_.transforms[0].header.stamp;
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

        createZedBasedTransform(collectiontfMessage_);

      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

  }  // end of for loop

  if (foundTfmsgs_) {
    for (auto& time : timeStamps) {
      if ((counter_ % 100 == 0) || (counter_ == 0)) {
        for (auto& transform : collectiontfMessage_.transforms) {
          transform.header.stamp = time;
        }

        outBag.write("/tf_static", time, collectiontfMessage_);
      }

      counter_++;
    }
  } else {
    ros::Time time = collectiontfMessage_.transforms[0].header.stamp;
    ros::Duration duration = ros::Duration(0.1);
    // collectiontfMessage_.transforms[1].header.stamp = time + duration;
    outBag.write("/tf_static", collectiontfMessage_.transforms[0].header.stamp, collectiontfMessage_);
    outBag.write("/tf_static", time + duration, collectiontfMessage_);
  }

  ROS_INFO("Finished running through the TF msgs.");
  // const ros::Time bag_begin_time = view.getBeginTime();
  // const ros::Time bag_end_time = view.getEndTime();
  endTime_ = std::chrono::steady_clock::now();

  // std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec() << " sec."
  //           << " Time elapsed for processing: " << elapsedSeconds() << " sec. \n \n";

  bag.close();
  outBag.close();
  return true;
}

void BoxPostProcessor::createZedBasedTransform(tf2_msgs::TFMessage& collectiontfMessage) {
  {
    geometry_msgs::TransformStamped zedToZed2iBase;
    zedToZed2iBase.header.frame_id = "zed_base_link";
    zedToZed2iBase.header.stamp = collectiontfMessage.transforms[0].header.stamp;
    zedToZed2iBase.child_frame_id = "zed2i_base_link";
    // conversion from Tranform to message

    tf2::Vector3 translation = tf2::Vector3(0.0, 0.0, 0.0);

    // X Y Z W
    tf2::Quaternion quat = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);

    zedToZed2iBase.transform.translation.x = translation.x();
    zedToZed2iBase.transform.translation.y = translation.y();
    zedToZed2iBase.transform.translation.z = translation.z();
    zedToZed2iBase.transform.rotation.x = quat.x();
    zedToZed2iBase.transform.rotation.y = quat.y();
    zedToZed2iBase.transform.rotation.z = quat.z();
    zedToZed2iBase.transform.rotation.w = quat.w();
    collectiontfMessage.transforms.push_back(zedToZed2iBase);
  }

  {
    ///////////////////////////////////////////////////////
    geometry_msgs::TransformStamped zed2iToZed2iCenter;
    zed2iToZed2iCenter.header.frame_id = "zed2i_base_link";
    zed2iToZed2iCenter.header.stamp = collectiontfMessage.transforms[0].header.stamp;
    zed2iToZed2iCenter.child_frame_id = "zed2i_camera_center";
    // conversion from Tranform to message

    tf2::Vector3 translation = tf2::Vector3(0.0, 0.0, 0.0);

    // X Y Z W
    tf2::Quaternion quat = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);

    zed2iToZed2iCenter.transform.translation.x = translation.x();
    zed2iToZed2iCenter.transform.translation.y = translation.y();
    zed2iToZed2iCenter.transform.translation.z = translation.z();
    zed2iToZed2iCenter.transform.rotation.x = quat.x();
    zed2iToZed2iCenter.transform.rotation.y = quat.y();
    zed2iToZed2iCenter.transform.rotation.z = quat.z();
    zed2iToZed2iCenter.transform.rotation.w = quat.w();
    collectiontfMessage.transforms.push_back(zed2iToZed2iCenter);
  }

  {
    ///////////////////////////////////////////////////////
    geometry_msgs::TransformStamped zed2iCenterToZed2iLeftCamera;
    zed2iCenterToZed2iLeftCamera.header.frame_id = "zed2i_camera_center";
    zed2iCenterToZed2iLeftCamera.header.stamp = collectiontfMessage.transforms[0].header.stamp;
    zed2iCenterToZed2iLeftCamera.child_frame_id = "zed2i_left_camera_frame";
    // conversion from Tranform to message

    tf2::Vector3 translation = tf2::Vector3(-0.01, 0.060, 0.0);

    // X Y Z W
    tf2::Quaternion quat = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);

    zed2iCenterToZed2iLeftCamera.transform.translation.x = translation.x();
    zed2iCenterToZed2iLeftCamera.transform.translation.y = translation.y();
    zed2iCenterToZed2iLeftCamera.transform.translation.z = translation.z();
    zed2iCenterToZed2iLeftCamera.transform.rotation.x = quat.x();
    zed2iCenterToZed2iLeftCamera.transform.rotation.y = quat.y();
    zed2iCenterToZed2iLeftCamera.transform.rotation.z = quat.z();
    zed2iCenterToZed2iLeftCamera.transform.rotation.w = quat.w();
    collectiontfMessage.transforms.push_back(zed2iCenterToZed2iLeftCamera);
  }

  {
    ///////////////////////////////////////////////////////
    geometry_msgs::TransformStamped zed2iCenterToZed2iRightCamera;
    zed2iCenterToZed2iRightCamera.header.frame_id = "zed2i_camera_center";
    zed2iCenterToZed2iRightCamera.header.stamp = collectiontfMessage.transforms[0].header.stamp;
    zed2iCenterToZed2iRightCamera.child_frame_id = "zed2i_right_camera_frame";
    // conversion from Tranform to message

    tf2::Vector3 translation = tf2::Vector3(-0.01, -0.060, 0.0);

    // X Y Z W
    tf2::Quaternion quat = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);

    zed2iCenterToZed2iRightCamera.transform.translation.x = translation.x();
    zed2iCenterToZed2iRightCamera.transform.translation.y = translation.y();
    zed2iCenterToZed2iRightCamera.transform.translation.z = translation.z();
    zed2iCenterToZed2iRightCamera.transform.rotation.x = quat.x();
    zed2iCenterToZed2iRightCamera.transform.rotation.y = quat.y();
    zed2iCenterToZed2iRightCamera.transform.rotation.z = quat.z();
    zed2iCenterToZed2iRightCamera.transform.rotation.w = quat.w();
    collectiontfMessage.transforms.push_back(zed2iCenterToZed2iRightCamera);
  }

  {
    ///////////////////////////////////////////////////////
    geometry_msgs::TransformStamped zed2iLeftCameraToZed2iIMU;
    zed2iLeftCameraToZed2iIMU.header.frame_id = "zed2i_left_camera_frame";
    zed2iLeftCameraToZed2iIMU.header.stamp = collectiontfMessage.transforms[0].header.stamp;
    zed2iLeftCameraToZed2iIMU.child_frame_id = "zed2i_imu_link";
    // conversion from Tranform to message

    tf2::Vector3 translation = tf2::Vector3(-0.002000, -0.023061, 0.000217);

    // X Y Z W
    tf2::Quaternion quat = tf2::Quaternion(-0.00183512, 0.000211432, -0.00651105, 0.999977);

    zed2iLeftCameraToZed2iIMU.transform.translation.x = translation.x();
    zed2iLeftCameraToZed2iIMU.transform.translation.y = translation.y();
    zed2iLeftCameraToZed2iIMU.transform.translation.z = translation.z();
    zed2iLeftCameraToZed2iIMU.transform.rotation.x = quat.x();
    zed2iLeftCameraToZed2iIMU.transform.rotation.y = quat.y();
    zed2iLeftCameraToZed2iIMU.transform.rotation.z = quat.z();
    zed2iLeftCameraToZed2iIMU.transform.rotation.w = quat.w();
    collectiontfMessage.transforms.push_back(zed2iLeftCameraToZed2iIMU);
  }
  {
    ///////////////////////////////////////////////////////
    geometry_msgs::TransformStamped zed2iLeftCameraToZed2iLeftOptical;
    zed2iLeftCameraToZed2iLeftOptical.header.frame_id = "zed2i_left_camera_frame";
    zed2iLeftCameraToZed2iLeftOptical.header.stamp = collectiontfMessage.transforms[0].header.stamp;
    zed2iLeftCameraToZed2iLeftOptical.child_frame_id = "zed2i_left_camera_optical_frame";
    // conversion from Tranform to message

    tf2::Vector3 translation = tf2::Vector3(0.0, 0.0, 0.0);

    // X Y Z W
    tf2::Quaternion quat = tf2::Quaternion(-0.500, 0.500, -0.500, 0.500);

    zed2iLeftCameraToZed2iLeftOptical.transform.translation.x = translation.x();
    zed2iLeftCameraToZed2iLeftOptical.transform.translation.y = translation.y();
    zed2iLeftCameraToZed2iLeftOptical.transform.translation.z = translation.z();
    zed2iLeftCameraToZed2iLeftOptical.transform.rotation.x = quat.x();
    zed2iLeftCameraToZed2iLeftOptical.transform.rotation.y = quat.y();
    zed2iLeftCameraToZed2iLeftOptical.transform.rotation.z = quat.z();
    zed2iLeftCameraToZed2iLeftOptical.transform.rotation.w = quat.w();
    collectiontfMessage.transforms.push_back(zed2iLeftCameraToZed2iLeftOptical);
  }

  {
    ///////////////////////////////////////////////////////
    geometry_msgs::TransformStamped zed2iRightCameraToZed2iRightOptical;
    zed2iRightCameraToZed2iRightOptical.header.frame_id = "zed2i_right_camera_frame";
    zed2iRightCameraToZed2iRightOptical.header.stamp = collectiontfMessage.transforms[0].header.stamp;
    zed2iRightCameraToZed2iRightOptical.child_frame_id = "zed2i_right_camera_optical_frame";
    // conversion from Tranform to message

    tf2::Vector3 translation = tf2::Vector3(0.0, 0.0, 0.0);

    // X Y Z W
    tf2::Quaternion quat = tf2::Quaternion(-0.500, 0.500, -0.500, 0.500);

    zed2iRightCameraToZed2iRightOptical.transform.translation.x = translation.x();
    zed2iRightCameraToZed2iRightOptical.transform.translation.y = translation.y();
    zed2iRightCameraToZed2iRightOptical.transform.translation.z = translation.z();
    zed2iRightCameraToZed2iRightOptical.transform.rotation.x = quat.x();
    zed2iRightCameraToZed2iRightOptical.transform.rotation.y = quat.y();
    zed2iRightCameraToZed2iRightOptical.transform.rotation.z = quat.z();
    zed2iRightCameraToZed2iRightOptical.transform.rotation.w = quat.w();
    collectiontfMessage.transforms.push_back(zed2iRightCameraToZed2iRightOptical);
  }
}

}  // namespace box_post_processor

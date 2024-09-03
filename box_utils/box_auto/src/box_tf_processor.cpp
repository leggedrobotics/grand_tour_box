#include "box_post_processor/box_tf_processor.hpp"
#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <filesystem>

namespace box_post_processor {

BoxTFProcessor::BoxTFProcessor(ros::NodeHandlePtr nh) : nh_(nh) {}

void BoxTFProcessor::initialize() {
  outputBagFolder_ = nh_->param<std::string>("output_folder", "");

  std::vector<std::string> tfContainingBags;

  nh_->getParam("/tf_containing_bag_paths", tfContainingBags);

  if (tfContainingBags.empty()) {
    ROS_ERROR_STREAM("No TF Input bag is provided.");
    ros::shutdown();
    return;
  }

  rosbagFullname_ = outputBagFolder_ + "/box_tf.bag";
  ROS_INFO_STREAM("Writing to Rosbag: " << rosbagFullname_);

  createOutputDirectory();
  if (std::filesystem::exists(rosbagFullname_)) {
    std::remove(rosbagFullname_.c_str());
  }
  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(2.0)};
  ros::WallTime::sleepUntil(first);

  if (processRosbags(tfContainingBags)) {
    ROS_INFO_STREAM("\033[92m"
                    << " SUCCESSFULLY COMPLETED REPLAYING. TERMINATING MYSELF. "
                    << "\033[0m");
    return;
  } else {
    ROS_ERROR_STREAM("Error in processing the Rosbag. Its your fault. You failed not the code... go grab a coffee.");
    ros::shutdown();
    return;
  }
}

bool BoxTFProcessor::createOutputDirectory() {
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

bool BoxTFProcessor::processRosbags(std::vector<std::string>& tfContainingBags) {
  std::vector<std::string> topics;
  topics.push_back("/tf");
  topics.push_back("/tf_static");

  std::vector<tf2_msgs::TFMessage> tfVector_;
  std::vector<tf2_msgs::TFMessage> tfStaticVector_;

  outBag.open(rosbagFullname_, rosbag::bagmode::Write);

  // Create me a high resolution clock timer from std chrono
  // Start the timer.
  startTime_ = std::chrono::steady_clock::now();

  // Iterate over the bags. Using tfContainingBags and a for loop

  ROS_INFO_STREAM("\033[92m"
                  << " Post processing the Rosbag "
                  << "\033[0m");
  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(1.0)};
  ros::WallTime::sleepUntil(first);

  for (const auto& bagPath : tfContainingBags) {
    std::string rosbagFilename_;
    rosbagFilename_ = bagPath;
    ROS_INFO_STREAM("Reading from Rosbag: " << rosbagFilename_);

    // Open ROS bag.
    rosbag::Bag bag;
    try {
      bag.open(rosbagFilename_, rosbag::bagmode::Read);
    } catch (const rosbag::BagIOException& e) {
      ROS_ERROR_STREAM("Error opening ROS bag: '" << rosbagFilename_ << "'");
      return false;
    }

    // The bag view we iterate over.
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::vector<ros::Time> timeStamps;
    bool foundAtopic = false;
    for (const auto& messageInstance : view) {
      // If the node is shutdown, stop processing and do early return.
      if (!ros::ok()) {
        return false;
      }

      if (messageInstance.getTopic() == "/tf") {
        foundAtopic = true;
        tf2_msgs::TFMessage::ConstPtr message = messageInstance.instantiate<tf2_msgs::TFMessage>();
        if (message != nullptr) {
          tfVector_.push_back(*message);
        }
      }

      if (messageInstance.getTopic() == "/tf_static") {
        foundAtopic = true;
        tf2_msgs::TFMessage::ConstPtr message = messageInstance.instantiate<tf2_msgs::TFMessage>();
        if (message != nullptr) {
          tfStaticVector_.push_back(*message);
        }
      }

      if (!foundAtopic) {
        ROS_ERROR_STREAM("No expected topic found in the bag.");
        bag.close();
        return false;
      }
    }

    bag.close();
  }

  if (tfVector_.empty() && tfStaticVector_.empty()) {
    ROS_ERROR_STREAM("No TF messages found in the bag.");
    return false;
  }

  // Iterate over the tfVector_ and tfStaticVector_ and write to the output bag.

  ROS_INFO_STREAM("Writing to Rosbag: " << rosbagFullname_);
  for (const auto& tfMessage : tfVector_) {
    if (tfMessage.transforms[0].header.stamp < ros::TIME_MIN) {
      ROS_ERROR_STREAM("The ROS time of the transform is invalid: " << tfMessage.transforms[0].header.stamp);
      return false;
    }

    outBag.write("/tf", tfMessage.transforms[0].header.stamp, tfMessage);
  }

  for (const auto& tfStaticMessage : tfStaticVector_) {
    if (tfStaticMessage.transforms[0].header.stamp < ros::TIME_MIN) {
      ROS_ERROR_STREAM("The ROS time of the transform is invalid: " << tfStaticMessage.transforms[0].header.stamp);
      return false;
    }

    outBag.write("/tf_static", tfStaticMessage.transforms[0].header.stamp, tfStaticMessage);
  }

  outBag.close();

  endTime_ = std::chrono::steady_clock::now();

  // std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec() << " sec."
  std::cout << "Elapsed Time (msec): " << elapsedMilliseconds() << " msec. \n \n";

  return true;
}

}  // namespace box_post_processor

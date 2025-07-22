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

#include "box_post_processor/box_imu_processor.hpp"
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <filesystem>

namespace box_post_processor {

BoxPostProcessor::BoxPostProcessor(ros::NodeHandlePtr nh) : nh_(nh) {}

void BoxPostProcessor::initialize() {
  inputRosbagFilename_ = nh_->param<std::string>("input_rosbag_file", "");
  if (inputRosbagFilename_.empty()) {
    throw std::runtime_error("Parameter 'input_rosbag_file' is not set or empty.");
  }

  outputRosbagFilename_ = nh_->param<std::string>("output_rosbag_file", "");
  if (outputRosbagFilename_.empty()) {
    throw std::runtime_error("Parameter 'output_rosbag_file' is not set or empty.");
  }

  time_offset_to_be_applied_ = nh_->param<double>("time_offset_to_apply", 0.0);

  mode_ = nh_->param<std::string>("mode", "");

  std::vector<std::string> param_names = {"adis", "stim", "livox"};

  // Check if mode is one of the allowed parameter names
  if (std::find(param_names.begin(), param_names.end(), mode_) == param_names.end()) {
    throw std::runtime_error("Parameter 'mode' must be one of: adis, stim, livox");
  }

  // Maps for storing parameter values for each mode

  for (const auto& param_name : param_names) {
    double maxTimeDifference = nh_->param<double>("/" + param_name + "/max_time_difference_millisecond", 0.0);
    if (maxTimeDifference < 0.0) {
      throw std::runtime_error("Parameter '" + param_name + "/max_time_difference_millisecond' is not set or negative.");
    }
    maxTimeDifferences_[param_name] = maxTimeDifference;

    // double timeOffsetToApply = nh_->param<double>("/" + param_name + "/time_offset_to_apply", -1000.0);
    // if (timeOffsetToApply < -100.0) {
    //   throw std::runtime_error("Parameter '" + param_name + "/time_offset_to_apply' is not set");
    // }
    // timeOffsetsToApply_[param_name] = timeOffsetToApply;

    XmlRpc::XmlRpcValue rpcTopic;
    if (nh_->getParam("/" + param_name + "/topics", rpcTopic)) {
      if (rpcTopic.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        allTopics_[param_name] = std::vector<std::string>();
        for (int i = 0; i < rpcTopic.size(); ++i) {
          std::string topic = static_cast<std::string>(rpcTopic[i]);
          allTopics_[param_name].push_back(topic);
        }
      } else {
        throw std::runtime_error("Parameter '" + param_name + "/topics' is not an array.");
      }
    } else {
      throw std::runtime_error("Failed to load parameter '" + param_name + "/topics'.");
    }
  }

  // Set variables based on selected mode
  maxTimeDifference_ = maxTimeDifferences_[mode_];
  // time_offset_to_be_applied_ = timeOffsetsToApply_[mode_];
  topics_ = allTopics_[mode_];

  ROS_INFO_STREAM("Time offset to be applied: " << time_offset_to_be_applied_ << " ms");
  ROS_INFO_STREAM("Maximum time offset: " << maxTimeDifference_ << " ms");
  ROS_INFO_STREAM("Reading from rosbag: " << inputRosbagFilename_);

  // createOutputDirectory();
  if (std::filesystem::exists(outputRosbagFilename_)) {
    std::remove(outputRosbagFilename_.c_str());
  }

  std::string fail_file_name = "/tmp/stim320_failed";
  if (std::filesystem::exists(fail_file_name)) {
    std::remove(fail_file_name.c_str());
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

// bool BoxPostProcessor::createOutputDirectory() {
//   // Check if the output folder exists.
//   if (!std::filesystem::is_directory(outputBagFolder_)) {
//     // If the folder doesn't exist, create it.
//     try {
//       return std::filesystem::create_directories(outputBagFolder_);
//     } catch (const std::exception& exception) {
//       ROS_ERROR_STREAM("Caught an exception trying to create output folder: " << exception.what());
//     }
//   }

//   return false;
// }

bool BoxPostProcessor::processRosbag() {
  // Create me a high resolution clock timer from std chrono
  // Start the timer.
  startTime_ = std::chrono::steady_clock::now();

  // Open ROS bag.
  rosbag::Bag bag;
  try {
    bag.open(inputRosbagFilename_, rosbag::bagmode::Read);
  } catch (const rosbag::BagIOException& e) {
    ROS_ERROR_STREAM("Error opening ROS bag: '" << inputRosbagFilename_ << "'");
    return false;
  }

  outBag.open(outputRosbagFilename_, rosbag::bagmode::Write);
  outBag.setCompression(rosbag::compression::LZ4);

  {
    std::set<std::string> availableTopics;
    rosbag::View allTopicsView(bag);
    for (const auto& m : allTopicsView) {
      availableTopics.insert(m.getTopic());
    }

    bool topicsFound = true;
    for (const auto& topic : topics_) {
      if (availableTopics.find(topic) == availableTopics.end()) {
        ROS_ERROR_STREAM("Topic '" << topic << "' is not available in the bag.");
        topicsFound = false;
      }
    }

    if (!topicsFound) {
      bag.close();
      return false;
    }
  }

  ROS_INFO_STREAM("\033[92m"
                  << " Post processing the Rosbag "
                  << "\033[0m");
  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(1.0)};
  ros::WallTime::sleepUntil(first);

  // The bag view we iterate over.
  rosbag::View bagView(bag, rosbag::TopicQuery(topics_));

  uint32_t errorCount = 0;
  uint32_t skipCount = 0;
  for (const auto& messageInstance : bagView) {
    // If the node is shutdown, stop processing and do early return.
    if (!ros::ok()) {
      return false;
    }

    if (messageInstance.getTopic().find("stim320/imu") != std::string::npos) {
      sensor_msgs::Imu::ConstPtr message = messageInstance.instantiate<sensor_msgs::Imu>();
      if (message != nullptr) {
        sensor_msgs::Imu newMessage = *message;

        ros::Time currentTime = message->header.stamp;

        uint64_t diff = currentTime.toNSec() - oldTime_.toNSec();

        if ((diff > maxTimeDifference_ * 1e6) && (oldTime_.toNSec() != 0u)) {
          ROS_ERROR_STREAM("Current Time: " << currentTime);
          ROS_ERROR_STREAM("oldTime_: " << oldTime_);
          ROS_ERROR_STREAM("Diff is to big: " << diff << " ns");
          errorCount++;
        }

        seq_ = message->header.seq;

        if ((seq_ != (oldSeq_ + 4)) && !((oldSeq_ == 252) && (seq_ == 0)) && (oldSeq_ != 300)) {
          ROS_ERROR_STREAM("Sequence Jump Seq: " << seq_ << " Old Seq: " << oldSeq_);
          skipCount++;
        }

        oldSeq_ = seq_;
        oldTime_ = currentTime;

        newMessage.header.stamp = currentTime + ros::Duration(time_offset_to_be_applied_ / 1000.0);

        outBag.write("/boxi/stim320/imu", newMessage.header.stamp, newMessage);
      }
    }

    if (messageInstance.getTopic().find("adis16475_node/imu") != std::string::npos) {
      sensor_msgs::Imu::ConstPtr message = messageInstance.instantiate<sensor_msgs::Imu>();
      if (message != nullptr) {
        sensor_msgs::Imu newMessage = *message;

        ros::Time currentTime = message->header.stamp;

        uint64_t diff = currentTime.toNSec() - oldTime_.toNSec();

        if ((diff > maxTimeDifference_ * 1e6) && (oldTime_.toNSec() != 0u)) {
          ROS_ERROR_STREAM("Current Time: " << currentTime);
          ROS_ERROR_STREAM("oldTime_: " << oldTime_);
          ROS_ERROR_STREAM("Diff is to big: " << diff << " ns");
          errorCount++;
        }

        oldTime_ = currentTime;

        newMessage.header.stamp = currentTime + ros::Duration(time_offset_to_be_applied_ / 1000.0);

        outBag.write("/boxi/adis16475_node/imu", newMessage.header.stamp, newMessage);
      }
    }

    if (messageInstance.getTopic().find("adis16475_node/imu") != std::string::npos) {
      sensor_msgs::Imu::ConstPtr message = messageInstance.instantiate<sensor_msgs::Imu>();
      if (message != nullptr) {
        sensor_msgs::Imu newMessage = *message;

        ros::Time currentTime = message->header.stamp;

        uint64_t diff = currentTime.toNSec() - oldTime_.toNSec();

        if ((diff > maxTimeDifference_ * 1e6) && (oldTime_.toNSec() != 0u)) {
          ROS_ERROR_STREAM("Current Time: " << currentTime);
          ROS_ERROR_STREAM("oldTime_: " << oldTime_);
          ROS_ERROR_STREAM("Diff is to big: " << diff << " ns");
          errorCount++;
        }

        oldTime_ = currentTime;

        newMessage.header.stamp = currentTime + ros::Duration(time_offset_to_be_applied_ / 1000.0);

        outBag.write("/boxi/adis16475_node/imu", newMessage.header.stamp, newMessage);
      }
    }

    if (messageInstance.getTopic().find("livox/imu") != std::string::npos) {
      sensor_msgs::Imu::ConstPtr message = messageInstance.instantiate<sensor_msgs::Imu>();
      if (message != nullptr) {
        sensor_msgs::Imu newMessage = *message;

        ros::Time currentTime = message->header.stamp;

        uint64_t diff = currentTime.toNSec() - oldTime_.toNSec();

        if ((diff > maxTimeDifference_ * 1e6) && (oldTime_.toNSec() != 0u)) {
          ROS_ERROR_STREAM("Current Time: " << currentTime);
          ROS_ERROR_STREAM("oldTime_: " << oldTime_);
          ROS_ERROR_STREAM("Diff is to big: " << diff << " ns");
          errorCount++;
        }

        oldTime_ = currentTime;

        newMessage.header.stamp = currentTime + ros::Duration(time_offset_to_be_applied_ / 1000.0);

        if (messageInstance.getTopic().find("compliant") == std::string::npos) {
          outBag.write("/boxi/livox/imu_si_compliant", newMessage.header.stamp, newMessage);
        } else {
          outBag.write("/boxi/livox/imu", newMessage.header.stamp, newMessage);
        }
      }
    }

    if (messageInstance.getTopic().find("stim320/acceletometer_temperature") != std::string::npos) {
      sensor_msgs::Temperature::ConstPtr message = messageInstance.instantiate<sensor_msgs::Temperature>();
      if (message != nullptr) {
        sensor_msgs::Temperature newMessage = *message;
        newMessage.header.stamp = message->header.stamp + ros::Duration(time_offset_to_be_applied_ / 1000.0);
        outBag.write("/boxi/stim320/gyroscope_temperature", newMessage.header.stamp, newMessage);
      }
    }

    if (messageInstance.getTopic().find("stim320/gyroscope_temperature") != std::string::npos) {
      sensor_msgs::Temperature::ConstPtr message = messageInstance.instantiate<sensor_msgs::Temperature>();
      if (message != nullptr) {
        sensor_msgs::Temperature newMessage = *message;
        newMessage.header.stamp = message->header.stamp + ros::Duration(time_offset_to_be_applied_ / 1000.0);
        outBag.write("/boxi/stim320/acceletometer_temperature", newMessage.header.stamp, newMessage);
      }
    }

    if (messageInstance.getDataType() == "sensor_msgs/PointCloud2") {
      sensor_msgs::PointCloud2::ConstPtr message = messageInstance.instantiate<sensor_msgs::PointCloud2>();
      if (message != nullptr) {
        sensor_msgs::PointCloud2 newMessage = *message;

        ros::Time currentTime = message->header.stamp;

        outBag.write(messageInstance.getTopic(), currentTime, newMessage);
      }
    }
  }

  ROS_INFO("Finished running through the msgs.");
  endTime_ = std::chrono::steady_clock::now();
  std::cout << "Elapsed Time (msec): " << elapsedMilliseconds() << " msec. \n \n";

  bag.close();
  outBag.close();

  if (skipCount > 0) {
    std::ofstream outFile("/tmp/stim320_failed");
    if (outFile) {
      outFile << "Skip Count: " << skipCount << std::endl;
    } else {
      ROS_ERROR("Failed to create file stim320_failed");
    }
  }

  return true;
}

}  // namespace box_post_processor

#include "box_post_processor/box_tf_processor.hpp"
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <regex>
#include <string>
#include <unordered_set>
#include <vector>

namespace box_post_processor {

BoxTFProcessor::BoxTFProcessor(ros::NodeHandlePtr nh) : nh_(nh) {}

// Combine transforms along a chain from startFrame to targetFrame.
// Returns true and sets outTf if a valid chain is found, otherwise returns false.
bool BoxTFProcessor::combineTransforms(tf2_ros::Buffer& tfBuffer, std::vector<tf2_msgs::TFMessage>& tfStatic, const std::string& startFrame,
                                       const std::string& targetFrame) {
  geometry_msgs::TransformStamped outTf;
  // Identity case: if startFrame == targetFrame, return an identity transform.
  if (startFrame == targetFrame) {
    outTf.header.frame_id = startFrame;
    outTf.child_frame_id = targetFrame;
    bool foundStamp = false;
    // Use tfStatic for a valid timestamp if available.
    for (const auto& staticMsg : tfStatic) {
      for (const auto& t : staticMsg.transforms) {
        if (t.header.frame_id == startFrame || t.child_frame_id == targetFrame) {
          outTf.header.stamp = t.header.stamp;
          foundStamp = true;
          break;
        }
      }
      if (foundStamp) break;
    }
    if (!foundStamp) {
      outTf.header.stamp = ros::Time(0);
    }
    tf2::Transform identity;
    identity.setIdentity();
    outTf.transform = tf2::toMsg(identity);
    return true;
  }

  // Non-identity case: use tfBuffer to retrieve the transform.
  try {
    // lookupTransform(targetFrame, startFrame, ...) returns a transform T such that:
    //   p_target = T * p_start
    // with header.frame_id = targetFrame and child_frame_id = startFrame.
    // Invert it so that the result has header.frame_id = startFrame and child_frame_id = targetFrame.
    geometry_msgs::TransformStamped tfStamped = tfBuffer.lookupTransform(targetFrame, startFrame, ros::Time(0));

    tf2::Transform tf;
    tf2::fromMsg(tfStamped.transform, tf);
    tf2::Transform tfInv = tf.inverse();

    outTf.header.frame_id = startFrame;
    outTf.child_frame_id = targetFrame;
    outTf.header.stamp = tfStamped.header.stamp;
    outTf.transform = tf2::toMsg(tfInv);

    // Add the computed transform to the tfStatic vector.
    tf2_msgs::TFMessage staticTfMsg;
    staticTfMsg.transforms.push_back(outTf);
    tfStatic.push_back(staticTfMsg);

    return true;
  } catch (tf2::TransformException& ex) {
    ROS_ERROR_STREAM("Failed to lookup transform from " << startFrame << " to " << targetFrame << ": " << ex.what());
    return false;
  }
}

void BoxTFProcessor::updateFrameNames(std::vector<tf2_msgs::TFMessage>& tfMsgs,
                                      const std::vector<std::tuple<std::string, std::string, bool>>& frameMapping) {
  for (auto& msg : tfMsgs) {
    std::vector<geometry_msgs::TransformStamped> newTransforms;

    for (auto& t : msg.transforms) {
      bool needsReplication = false;
      geometry_msgs::TransformStamped replicatedTransform = t;

      for (const auto& mapping : frameMapping) {
        const std::string& newName = std::get<0>(mapping);
        const std::string& oldName = std::get<1>(mapping);
        const bool replicate = std::get<2>(mapping);

        if (t.header.frame_id == oldName) {
          if (replicate) {
            needsReplication = true;
            replicatedTransform.header.frame_id = newName;
          } else {
            t.header.frame_id = newName;
          }
        }

        if (t.child_frame_id == oldName) {
          if (replicate) {
            needsReplication = true;
            replicatedTransform.child_frame_id = newName;
          } else {
            t.child_frame_id = newName;
          }
        }
      }

      if (needsReplication) {
        newTransforms.push_back(replicatedTransform);
      }
    }

    // Add all replicated transforms to the message
    msg.transforms.insert(msg.transforms.end(), newTransforms.begin(), newTransforms.end());
  }
}

std::string BoxTFProcessor::extractDatePrefix(const std::string& globalPath) {
  std::filesystem::path p(globalPath);
  std::string filename = p.filename().string();

  // Regex to match a date prefix at the start of the filename.
  // Expected format: YYYY-MM-DD-HH-MM-SS
  std::regex dateRegex(R"(^(\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}))");
  std::smatch match;
  if (std::regex_search(filename, match, dateRegex)) {
    return match[1].str();
  } else {
    // If not found, you might want to handle the error; here we return an empty string.
    return "";
  }
}

// Function that constructs the output bag name by appending the provided suffix
// and ensuring the date prefix is at the beginning.
std::string BoxTFProcessor::createOutputBagName(const std::string& inputGlobalPath, const std::string& outputSuffix) {
  std::string datePrefix = extractDatePrefix(inputGlobalPath);
  if (datePrefix.empty()) {
    // Fall back to the filename stem if no date prefix is found.
    std::filesystem::path p(inputGlobalPath);
    datePrefix = p.stem().string();
  }

  ROS_INFO_STREAM("datePrefix: " << datePrefix);

  return datePrefix + "_" + outputSuffix;
}

bool BoxTFProcessor::loadBoxTfParameters(ros::NodeHandle& nh, BoxTfParams& params) {
  XmlRpc::XmlRpcValue xmlMapping;
  if (nh.getParam("/frameMapping", xmlMapping)) {
    if (xmlMapping.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < xmlMapping.size(); ++i) {
        if (xmlMapping[i].hasMember("new") && xmlMapping[i].hasMember("old")) {
          std::string new_frame = static_cast<std::string>(xmlMapping[i]["new"]);
          std::string old_frame = static_cast<std::string>(xmlMapping[i]["old"]);
          bool replicate = static_cast<bool>(xmlMapping[i]["replicate"]);
          if (new_frame.empty() || old_frame.empty()) {
            ROS_WARN("framePairs entry %d has empty 'new_frame' or 'old_frame' value. Skipping.", i);
            continue;
          }
          params.frameMapping.push_back(std::make_tuple(new_frame, old_frame, replicate));
        } else {
          ROS_WARN("frameMapping entry %d is missing 'new' or 'old' keys.", i);
        }
      }
    } else {
      ROS_ERROR("Parameter 'frameMapping' is not an array.");
      return false;
    }
  } else {
    ROS_ERROR("Failed to load parameter 'frameMapping'.");
    return false;
  }

  // Load framePairs
  XmlRpc::XmlRpcValue xmlPairs;
  if (nh.getParam("/framePairs", xmlPairs)) {
    if (xmlPairs.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < xmlPairs.size(); ++i) {
        if (xmlPairs[i].hasMember("parent") && xmlPairs[i].hasMember("child")) {
          std::string parent = static_cast<std::string>(xmlPairs[i]["parent"]);
          std::string child = static_cast<std::string>(xmlPairs[i]["child"]);
          // Skip entries with empty parent or child frames
          if (parent.empty() || child.empty()) {
            ROS_WARN("framePairs entry %d has empty 'parent' or 'child' value. Skipping.", i);
            continue;
          }
          params.framePairs.push_back(std::make_pair(parent, child));
        } else {
          ROS_WARN("framePairs entry %d is missing 'parent' or 'child' keys.", i);
        }
      }
    } else {
      ROS_ERROR("Parameter 'framePairs' is not an array.");
      return false;
    }
  } else {
    ROS_ERROR("Failed to load parameter 'framePairs'.");
    return false;
  }

  // Load childFramesToRemove
  XmlRpc::XmlRpcValue xmlChildRemove;
  if (nh.getParam("/childFramesToRemove", xmlChildRemove)) {
    if (xmlChildRemove.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < xmlChildRemove.size(); ++i) {
        std::string frame = static_cast<std::string>(xmlChildRemove[i]);
        params.childFramesToRemove.push_back(frame);
      }
    } else {
      ROS_ERROR("Parameter 'childFramesToRemove' is not an array.");
      return false;
    }
  } else {
    ROS_ERROR("Failed to load parameter 'childFramesToRemove'.");
    return false;
  }

  // Load parentFramesToRemove
  XmlRpc::XmlRpcValue xmlParentRemove;
  if (nh.getParam("/parentFramesToRemove", xmlParentRemove)) {
    if (xmlParentRemove.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < xmlParentRemove.size(); ++i) {
        std::string frame = static_cast<std::string>(xmlParentRemove[i]);
        params.parentFramesToRemove.push_back(frame);
      }
    } else {
      ROS_ERROR("Parameter 'parentFramesToRemove' is not an array.");
      return false;
    }
  } else {
    ROS_ERROR("Failed to load parameter 'parentFramesToRemove'.");
    return false;
  }

  // Load parentFramesToRemove
  XmlRpc::XmlRpcValue xmlChildToInverse;
  if (nh.getParam("/childFramesToInverse", xmlChildToInverse)) {
    if (xmlChildToInverse.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < xmlChildToInverse.size(); ++i) {
        std::string frame = static_cast<std::string>(xmlChildToInverse[i]);
        params.childFramesToInverse.push_back(frame);
      }
    } else {
      ROS_ERROR("Parameter 'childFramesToInverse' is not an array.");
      return false;
    }
  } else {
    ROS_ERROR("Failed to load parameter 'childFramesToInverse'.");
    return false;
  }

  XmlRpc::XmlRpcValue xmlRotationOverrides;
  if (nh.getParam("/rotationOverrides", xmlRotationOverrides)) {
    if (xmlRotationOverrides.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < xmlRotationOverrides.size(); ++i) {
        if (xmlRotationOverrides[i].hasMember("parent") && xmlRotationOverrides[i].hasMember("child") &&
            xmlRotationOverrides[i].hasMember("axis") && xmlRotationOverrides[i].hasMember("angle_deg")) {
          std::string parent = static_cast<std::string>(xmlRotationOverrides[i]["parent"]);
          std::string child = static_cast<std::string>(xmlRotationOverrides[i]["child"]);
          std::string axis = static_cast<std::string>(xmlRotationOverrides[i]["axis"]);
          double angle_deg = static_cast<double>(xmlRotationOverrides[i]["angle_deg"]);
          params.rotationOverrides.push_back({parent, child, axis, angle_deg});
        } else {
          ROS_WARN("rotationOverrides entry %d is missing required keys.", i);
        }
      }
    } else {
      ROS_ERROR("Parameter 'rotationOverrides' is not an array.");
      return false;
    }
  } else {
    ROS_INFO("No rotationOverrides parameter provided.");
  }

  return true;
}

void BoxTFProcessor::initialize() {
  outputBagFolder_ = nh_->param<std::string>("output_folder", "");
  std::string outputSuffix = nh_->param<std::string>("bag_post_fix", "");
  tfStaticRepetitionPeriod_ = nh_->param<double>("/tf_static_repetition_period", 5.0);

  if (loadBoxTfParameters(*nh_, params)) {
    ROS_INFO("Successfully loaded box_tf parameters.");

    ROS_INFO("Loaded frameMapping: %zu entries", params.frameMapping.size());
    ROS_INFO("Loaded framePairs: %zu entries", params.framePairs.size());
    ROS_INFO("Loaded childFramesToRemove: %zu entries", params.childFramesToRemove.size());
    ROS_INFO("Loaded parentFramesToRemove: %zu entries", params.parentFramesToRemove.size());
    ROS_INFO("Loaded rotationOverride: %zu entries", params.rotationOverrides.size());
  } else {
    ROS_ERROR("Failed to load box_tf parameters. Exitting.");
    ros::shutdown();
  }

  std::vector<std::string> tfContainingBags;

  nh_->getParam("/tf_containing_bag_paths", tfContainingBags);

  if (tfContainingBags.empty()) {
    ROS_ERROR_STREAM("No TF Input bag is provided.");
    ros::shutdown();
    return;
  }

  if (outputSuffix.size() < 4 || outputSuffix.substr(outputSuffix.size() - 4) != ".bag") {
    outputSuffix += ".bag";
  }

  // Create the output bag name.
  std::string outputBag = createOutputBagName(tfContainingBags[0], outputSuffix);

  std::filesystem::path outputDir(outputBagFolder_);
  std::filesystem::path outputFile(outputBag);
  rosbagFullname_ = (outputDir / outputFile).string();

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

void BoxTFProcessor::applyRotationOverrides(std::vector<tf2_msgs::TFMessage>& tfStaticMsgs) {
  // Early return if tfStaticMsgs is empty.
  if (tfStaticMsgs.empty()) {
    return;
  }

  bool foundAtLeastOne = false;
  for (auto& msg : tfStaticMsgs) {
    for (auto& transform : msg.transforms) {
      for (const auto& override : params.rotationOverrides) {
        if (transform.header.frame_id == override.parent_frame && transform.child_frame_id == override.child_frame) {
          foundAtLeastOne = true;

          // Convert the current rotation to a tf2 quaternion.
          tf2::Quaternion q_orig;
          tf2::fromMsg(transform.transform.rotation, q_orig);

          // Convert the current translation to a tf2 vector.
          tf2::Vector3 t_orig(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);

          // Convert the provided angle from degrees to radians.
          double angle_rad = override.angle_deg * M_PI / 180.0;
          tf2::Quaternion q_add;
          if (override.axis == "x" || override.axis == "X") {
            q_add.setRPY(angle_rad, 0, 0);
          } else if (override.axis == "y" || override.axis == "Y") {
            q_add.setRPY(0, angle_rad, 0);
          } else if (override.axis == "z" || override.axis == "Z") {
            q_add.setRPY(0, 0, angle_rad);
          } else {
            ROS_WARN_STREAM("Invalid axis: " << override.axis << " for rotation override, skipping.");
            continue;
          }

          // Compute the new rotation: additional override applied to the original.
          tf2::Quaternion q_new = q_add * q_orig;
          q_new.normalize();
          transform.transform.rotation = tf2::toMsg(q_new);

          // Apply the rotation override to the translation vector.
          // This rotates the translation vector by q_add.
          tf2::Vector3 t_new = tf2::quatRotate(q_add, t_orig);
          transform.transform.translation.x = t_new.x();
          transform.transform.translation.y = t_new.y();
          transform.transform.translation.z = t_new.z();
        }
      }
    }
  }

  if (!foundAtLeastOne) {
    ROS_ERROR("No rotation overrides applied.");
    throw std::runtime_error("No rotation overrides were applied despite having override configurations.");
  }
}

bool BoxTFProcessor::processRosbags(std::vector<std::string>& tfContainingBags) {
  std::vector<std::string> topics;
  topics.push_back("/tf");
  topics.push_back("/tf_static");

  std::vector<tf2_msgs::TFMessage> tfVector_;
  std::vector<tf2_msgs::TFMessage> tfStaticVector_;

  outBag.open(rosbagFullname_, rosbag::bagmode::Write);
  outBag.setCompression(rosbag::compression::LZ4);

  // Create me a high resolution clock timer from std chrono
  // Start the timer.
  startTime_ = std::chrono::steady_clock::now();

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
          if (message->transforms.empty()) {
            continue;  // Skip if the message does not contain any transforms
          }
          tfVector_.push_back(*message);
        }
      }

      if (messageInstance.getTopic() == "/tf_static") {
        foundAtopic = true;
        tf2_msgs::TFMessage::ConstPtr message = messageInstance.instantiate<tf2_msgs::TFMessage>();
        if (message != nullptr) {
          if (message->transforms.empty()) {
            continue;  // Skip if the message does not contain any transforms
          }
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

  // Sort tfVector_ by timestamp of the first transform in each message
  std::sort(tfVector_.begin(), tfVector_.end(), [](const tf2_msgs::TFMessage& a, const tf2_msgs::TFMessage& b) {
    if (a.transforms.empty()) return false;
    if (b.transforms.empty()) return true;
    return a.transforms.front().header.stamp < b.transforms.front().header.stamp;
  });

  // Sort tfStaticVector_ by timestamp of the first transform in each message
  std::sort(tfStaticVector_.begin(), tfStaticVector_.end(), [](const tf2_msgs::TFMessage& a, const tf2_msgs::TFMessage& b) {
    if (a.transforms.empty()) return false;
    if (b.transforms.empty()) return true;
    return a.transforms.front().header.stamp < b.transforms.front().header.stamp;
  });

  // Additionally, sort transforms within each message by timestamp
  for (auto& msg : tfVector_) {
    std::sort(
        msg.transforms.begin(), msg.transforms.end(),
        [](const geometry_msgs::TransformStamped& a, const geometry_msgs::TransformStamped& b) { return a.header.stamp < b.header.stamp; });
  }

  for (auto& msg : tfStaticVector_) {
    std::sort(
        msg.transforms.begin(), msg.transforms.end(),
        [](const geometry_msgs::TransformStamped& a, const geometry_msgs::TransformStamped& b) { return a.header.stamp < b.header.stamp; });
  }

  ros::Time firstTFTime = tfVector_.front().transforms.front().header.stamp - ros::Duration(1.0);
  ros::Time lastTFTime = tfVector_.back().transforms.back().header.stamp;
  ROS_INFO_STREAM("First TF timestamp: " << firstTFTime);
  ROS_INFO_STREAM("Last TF timestamp: " << lastTFTime);

  // Find earliest tf_static timestamp
  ros::Time earliestTfStaticTime;
  if (!tfStaticVector_.empty() && !tfStaticVector_.front().transforms.empty()) {
    earliestTfStaticTime = tfStaticVector_.front().transforms.front().header.stamp;
    ROS_INFO_STREAM("Earliest TF static timestamp: " << earliestTfStaticTime);

    // Check if difference is more than 10 seconds
    double timeDiff = std::abs((earliestTfStaticTime - firstTFTime).toSec());
    if (timeDiff > 10.0) {
      ROS_WARN_STREAM("Time difference between earliest tf_static and tf is "
                      << timeDiff << " seconds (>10s). Adjusting tf_static timestamps to match earliest tf time.");

      // Adjust all tf_static timestamps to match earliest tf time
      for (auto& msg : tfStaticVector_) {
        for (auto& transform : msg.transforms) {
          transform.header.stamp = firstTFTime;
        }
      }
      ROS_INFO_STREAM("All tf_static timestamps adjusted to: " << firstTFTime);
    }
  } else {
    ROS_INFO_STREAM("No tf_static transforms found.");
  }

  tf2_ros::Buffer tfBuffer_(ros::Duration(lastTFTime.toSec() - firstTFTime.toSec() + 500.0));
  for (const auto& staticMsg : tfStaticVector_) {
    for (const auto& transform : staticMsg.transforms) {
      tfBuffer_.setTransform(transform, "default_authority", true);
    }
  }

  for (const auto& tfMsg : tfVector_) {
    for (const auto& transform : tfMsg.transforms) {
      tfBuffer_.setTransform(transform, "default_authority", false);
    }
  }

  // List of child frame ids to remove
  if (params.childFramesToInverse.empty()) {
    ROS_INFO_STREAM("No child frames to inverse.");
  } else {
    // Inverse transforms whose child frame is in childFramesToInverse
    {
      std::unordered_set<std::string> framesToInverse(params.childFramesToInverse.begin(), params.childFramesToInverse.end());
      auto inverseTransforms = [&framesToInverse](auto& tfVector) {
        for (auto& tfMessage : tfVector) {
          for (auto& transform : tfMessage.transforms) {
            if (framesToInverse.find(transform.child_frame_id) != framesToInverse.end()) {
              // Swap header and child frame IDs
              std::swap(transform.header.frame_id, transform.child_frame_id);

              // Invert the transform using tf2
              tf2::Transform tf_transform;
              tf2::fromMsg(transform.transform, tf_transform);
              tf_transform = tf_transform.inverse();
              transform.transform = tf2::toMsg(tf_transform);
            }
          }
        }
      };

      inverseTransforms(tfVector_);
      inverseTransforms(tfStaticVector_);
    }
  }

  // Add all child frames from framePairs to childFramesToRemove
  for (const auto& pair : params.framePairs) {
    const std::string& childFrame = pair.second;

    // Check if the child frame is already in childFramesToRemove
    if (std::find(params.childFramesToRemove.begin(), params.childFramesToRemove.end(), childFrame) == params.childFramesToRemove.end()) {
      // If not present, add it
      params.childFramesToRemove.push_back(childFrame);
      ROS_INFO_STREAM("Added " << childFrame << " to childFramesToRemove");
    }
  }

  // Remove transforms with child frame ids in childFramesToRemove
  {
    std::unordered_set<std::string> framesToRemove(params.childFramesToRemove.begin(), params.childFramesToRemove.end());

    auto filterTransforms = [&framesToRemove](auto& tfVector) {
      for (auto& tfMessage : tfVector) {
        auto& transforms = tfMessage.transforms;
        transforms.erase(std::remove_if(transforms.begin(), transforms.end(),
                                        [&framesToRemove](const geometry_msgs::TransformStamped& transform) {
                                          return framesToRemove.find(transform.child_frame_id) != framesToRemove.end();
                                        }),
                         transforms.end());
      }
    };

    filterTransforms(tfVector_);
    filterTransforms(tfStaticVector_);
  }

  // Remove transforms whose parent (header.frame_id) is in parentFramesToRemove
  {
    std::unordered_set<std::string> parentFramesSet(params.parentFramesToRemove.begin(), params.parentFramesToRemove.end());

    auto filterByParent = [&parentFramesSet](auto& tfMessages) {
      for (auto& msg : tfMessages) {
        auto& transforms = msg.transforms;
        transforms.erase(std::remove_if(transforms.begin(), transforms.end(),
                                        [&parentFramesSet](const geometry_msgs::TransformStamped& t) {
                                          return parentFramesSet.find(t.header.frame_id) != parentFramesSet.end();
                                        }),
                         transforms.end());
      }
    };

    filterByParent(tfVector_);
    filterByParent(tfStaticVector_);
  }

  // First, compute all combinations without modifying tfVector.
  for (const auto& pair : params.framePairs) {
    // geometry_msgs::TransformStamped combinedTf;
    // std::vector<const geometry_msgs::TransformStamped*> chain; // Will remain empty.
    if (combineTransforms(tfBuffer_, tfStaticVector_, pair.first, pair.second)) {
      ROS_INFO_STREAM("Combined transform from " << pair.first << " to " << pair.second << " computed.");
      // combinedResults.push_back({combinedTf, chain});
    } else {
      ROS_ERROR_STREAM("Failed to combine transforms from " << pair.first << " to " << pair.second);
    }
  }

  if (!params.frameMapping.empty()) {
    // --- Update the Frame Names ---
    updateFrameNames(tfStaticVector_, params.frameMapping);
    updateFrameNames(tfVector_, params.frameMapping);
  } else {
    ROS_INFO_STREAM("No frame mapping provided.");
  }

  if (!params.rotationOverrides.empty()) {
    // --- Apply Rotation Overrides ---
    // Apply pre-defined rotations
    applyRotationOverrides(tfStaticVector_);

    // Special case: Find transforms with parent "Y" and child "X" and negate translation
    {
      bool foundSpecialTransform = false;
      for (auto& tfStaticMsg : tfStaticVector_) {
        for (auto& transform : tfStaticMsg.transforms) {
          if (transform.header.frame_id == "zed2i_left_camera_optical_frame" &&
              transform.child_frame_id == "zed2i_right_camera_optical_frame") {
            // Multiply translation components by -1
            transform.transform.translation.x *= -1;
            transform.transform.translation.y *= -1;
            transform.transform.translation.z *= -1;

            foundSpecialTransform = true;
            ROS_INFO_STREAM("Negated translation for transform from zed2i_left_camera_optical_frame to zed2i_right_camera_optical_frame");
          }
        }
      }

      if (!foundSpecialTransform) {
        ROS_WARN_STREAM("No transforms found with parent 'zed2i_left_camera_optical_frame' and child 'zed2i_right_camera_optical_frame'");
      }
    }
  } else {
    ROS_INFO_STREAM("No rotation overrides provided.");
  }

  {
    // Find transforms from stim320_imu to box_base_model and set them to identity with frame_id = box_base
    for (auto& tfStaticMsg : tfStaticVector_) {
      for (auto& transform : tfStaticMsg.transforms) {
        if (transform.header.frame_id == "stim320_imu" && transform.child_frame_id == "box_base_model") {
          // Change the frame_id to box_base
          transform.header.frame_id = "box_base";

          // Set transform to identity
          tf2::Transform identity;
          identity.setIdentity();
          transform.transform = tf2::toMsg(identity);

          ROS_WARN_STREAM_THROTTLE(0.5,"####### OLD TF IS GETTING MERGED #######");
          ROS_WARN_STREAM_THROTTLE(0.5,"Changed transform: stim320_imu -> box_base_model to identity transform from box_base -> box_base_model");
          ROS_WARN_STREAM_THROTTLE(0.5,"####### OLD TF IS GETTING MERGED #######");
        }
      }
    }
  }

  {
    // Find the earliest timestamp in tfStaticVector_
    ros::Time earliestStaticTime = firstTFTime;  // Default to firstTFTime
    bool foundValidTime = false;

    ROS_INFO_STREAM("Writing tf_static transforms with timestamp: " << earliestStaticTime);

    // Combine all static transforms into a single message
    tf2_msgs::TFMessage combinedStaticMsg;
    std::set<std::pair<std::string, std::string>> seenPairs;

    // Collect all unique transforms from all static messages
    for (const auto& tfStaticMsg : tfStaticVector_) {
      for (const auto& transform : tfStaticMsg.transforms) {
        // Create a pair representing this parent-child relationship
        std::pair<std::string, std::string> framePair(transform.header.frame_id, transform.child_frame_id);

        // Only add this transform if we haven't seen this parent-child pair before
        if (seenPairs.find(framePair) == seenPairs.end()) {
          // Add to the combined message with the earliest timestamp
          geometry_msgs::TransformStamped updatedTransform = transform;
          updatedTransform.header.stamp = earliestStaticTime;
          combinedStaticMsg.transforms.push_back(updatedTransform);
          seenPairs.insert(framePair);
        }
      }
    }

    // Write the combined static message periodically
    if (!combinedStaticMsg.transforms.empty()) {
      ros::Time startTime = earliestStaticTime - ros::Duration(1.0);
      ROS_INFO_STREAM("Writing combined tf_static message with "
                      << combinedStaticMsg.transforms.size() << " unique transforms periodically from " << startTime << " to " << lastTFTime
                      << " every " << tfStaticRepetitionPeriod_ << " seconds");

      // Repeat the tf_static transforms at regular intervals
      for (ros::Time t = startTime; t <= lastTFTime; t += ros::Duration(tfStaticRepetitionPeriod_)) {
        // Update all transforms to have the current timestamp
        for (auto& transform : combinedStaticMsg.transforms) {
          transform.header.stamp = t;
        }
        outBag.write("/tf_static", t, combinedStaticMsg);
      }
    } else {
      ROS_WARN_STREAM("No static transforms found to write");
    }
  }

  ROS_INFO_STREAM("Writing to Rosbag: " << rosbagFullname_);
  for (const auto& tfMessage : tfVector_) {
    // Skip if there are no transforms in this message
    if (tfMessage.transforms.empty()) {
      continue;
    }

    if (tfMessage.transforms[0].header.stamp < ros::TIME_MIN) {
      ROS_ERROR_STREAM("The ROS time of the transform is invalid: " << tfMessage.transforms[0].header.stamp);
      return false;
    }

    outBag.write("/tf", tfMessage.transforms[0].header.stamp, tfMessage);
  }

  outBag.close();
  endTime_ = std::chrono::steady_clock::now();

  std::cout << "Elapsed Time (msec): " << elapsedMilliseconds() << " msec. \n \n";

  return true;
}

}  // namespace box_post_processor

#include "box_post_processor/box_tf_processor.hpp"
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
bool BoxTFProcessor::combineTransforms(std::vector<tf2_msgs::TFMessage>& tfMsgs, const std::string& startFrame,
                                       const std::string& targetFrame, geometry_msgs::TransformStamped& outTf,
                                       std::vector<const geometry_msgs::TransformStamped*>& chainOut) {
  chainOut.clear();

  // Identity case: if startFrame == targetFrame, return an identity transform.
  if (startFrame == targetFrame) {
    outTf.header.frame_id = startFrame;
    outTf.child_frame_id = targetFrame;
    bool foundStamp = false;
    for (const auto& msg : tfMsgs) {
      for (const auto& t : msg.transforms) {
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

  // Build a graph: map each parent frame to pointers of outgoing transforms.
  std::unordered_map<std::string, std::vector<const geometry_msgs::TransformStamped*>> graph;
  for (auto& msg : tfMsgs) {
    for (auto& t : msg.transforms) {
      graph[t.header.frame_id].push_back(&t);
    }
  }

  // BFS: Each node is a pair (current frame, chain of transform pointers used so far).
  using Node = std::pair<std::string, std::vector<const geometry_msgs::TransformStamped*>>;
  std::queue<Node> q;
  q.push({startFrame, {}});
  std::unordered_set<std::string> visited;
  visited.insert(startFrame);

  bool found = false;
  std::vector<const geometry_msgs::TransformStamped*> chainFound;

  while (!q.empty()) {
    auto [frame, chain] = q.front();
    q.pop();

    if (frame == targetFrame) {
      chainFound = chain;
      found = true;
      break;
    }

    if (graph.count(frame)) {
      for (const auto* t_ptr : graph[frame]) {
        const std::string& nextFrame = t_ptr->child_frame_id;
        if (visited.insert(nextFrame).second) {
          auto newChain = chain;
          newChain.push_back(t_ptr);
          q.push({nextFrame, std::move(newChain)});
        }
      }
    }
  }

  if (!found) {
    ROS_ERROR_STREAM("No valid transform chain found from " << startFrame << " to " << targetFrame);
    return false;
  }

  // Compose the transforms along the found chain.
  tf2::Transform combined;
  combined.setIdentity();
  for (const auto* t_ptr : chainFound) {
    tf2::Transform tf;
    try {
      tf2::fromMsg(t_ptr->transform, tf);
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Error converting transform: " << e.what());
      return false;
    }
    combined *= tf;
  }

  outTf.header.frame_id = startFrame;
  outTf.child_frame_id = targetFrame;
  // Use the header timestamp of the first transform in the chain.
  if (!chainFound.empty()) {
    outTf.header.stamp = chainFound.front()->header.stamp;
  } else {
    outTf.header.stamp = ros::Time(0);
  }
  outTf.transform = tf2::toMsg(combined);
  chainOut = chainFound;
  return true;
}

void BoxTFProcessor::updateFrameNames(std::vector<tf2_msgs::TFMessage>& tfMsgs,
                                      const std::vector<std::pair<std::string, std::string>>& frameMapping) {
  for (auto& msg : tfMsgs) {
    for (auto& t : msg.transforms) {
      for (const auto& mapping : frameMapping) {
        const std::string& newName = mapping.first;
        const std::string& oldName = mapping.second;
        if (t.header.frame_id == oldName) {
          t.header.frame_id = newName;
        }
        if (t.child_frame_id == oldName) {
          t.child_frame_id = newName;
        }
      }
    }
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
          params.frameMapping.push_back(std::make_pair(new_frame, old_frame));
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

  ros::Time firstTFTime = tfVector_.front().transforms.front().header.stamp - ros::Duration(1.0);
  ros::Time lastTFTime = tfVector_.back().transforms.back().header.stamp;
  ROS_INFO_STREAM("First TF timestamp: " << firstTFTime);
  ROS_INFO_STREAM("Last TF timestamp: " << lastTFTime);

  {
    // Merge all tfStatic messages into one message.
    if (!tfStaticVector_.empty()) {
      tf2_msgs::TFMessage mergedStaticMsg;
      for (const auto& msg : tfStaticVector_) {
        mergedStaticMsg.transforms.insert(mergedStaticMsg.transforms.end(), msg.transforms.begin(), msg.transforms.end());
      }
      tfStaticVector_.clear();
      tfStaticVector_.push_back(mergedStaticMsg);
    }
  }

  if (!tfVector_.empty() && !tfVector_[0].transforms.empty()) {
    for (auto& tfStaticMsg : tfStaticVector_) {
      for (auto& transform : tfStaticMsg.transforms) {
        transform.header.stamp = firstTFTime;
      }
    }
  } else {
    ROS_ERROR_STREAM("tfVector_ is empty or has no valid transforms to extract time.");
    return false;
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

  // We'll store each combined transform along with its chain for later removal.
  std::vector<std::pair<geometry_msgs::TransformStamped, std::vector<const geometry_msgs::TransformStamped*>>> combinedResults;

  // First, compute all combinations without modifying tfVector.
  for (const auto& pair : params.framePairs) {
    geometry_msgs::TransformStamped combinedTf;
    std::vector<const geometry_msgs::TransformStamped*> chain;
    if (combineTransforms(tfStaticVector_, pair.first, pair.second, combinedTf, chain)) {
      ROS_INFO_STREAM("Combined transform from " << pair.first << " to " << pair.second << " computed.");
      combinedResults.push_back({combinedTf, chain});
    } else {
      ROS_ERROR_STREAM("Failed to combine transforms from " << pair.first << " to " << pair.second);
    }
  }

  // Now, remove all individual transforms that are part of any chain.
  std::unordered_set<const geometry_msgs::TransformStamped*> removalSet;
  for (const auto& entry : combinedResults) {
    for (const auto* ptr : entry.second) {
      removalSet.insert(ptr);
    }
  }
  for (auto& msg : tfStaticVector_) {
    msg.transforms.erase(
        std::remove_if(msg.transforms.begin(), msg.transforms.end(),
                       [&removalSet](const geometry_msgs::TransformStamped& t) { return removalSet.find(&t) != removalSet.end(); }),
        msg.transforms.end());
  }

  // Add all combined transforms as new tf2_msgs::TFMessage entries.
  for (const auto& entry : combinedResults) {
    tf2_msgs::TFMessage newMsg;
    newMsg.transforms.push_back(entry.first);
    tfStaticVector_.push_back(newMsg);
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

  // --- Update the Frame Names ---
  updateFrameNames(tfStaticVector_, params.frameMapping);

  {
    ROS_INFO_STREAM("Repeating tf_static transforms every " << tfStaticRepetitionPeriod_ << " second(s) from " << firstTFTime << " to "
                                                            << lastTFTime);

    // Repeat the tf_static transforms every "tfStaticRepetitionPeriod_" seconds between first and last times.
    for (ros::Time t = firstTFTime; t <= lastTFTime; t += ros::Duration(tfStaticRepetitionPeriod_)) {
      for (const auto& tfStaticMsg : tfStaticVector_) {
        // Create a modified copy with updated header times.
        tf2_msgs::TFMessage repeatedMsg = tfStaticMsg;
        for (auto& transform : repeatedMsg.transforms) {
          transform.header.stamp = t;
        }
        outBag.write("/tf_static", t, repeatedMsg);
      }
    }
  }

  ROS_INFO_STREAM("Writing to Rosbag: " << rosbagFullname_);
  for (const auto& tfMessage : tfVector_) {
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

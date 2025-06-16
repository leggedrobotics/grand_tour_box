#include <XmlRpcValue.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

// Define a struct to store each triplet
struct PrepPrameters {
  std::string bag = "";
  std::string topic = "";
  std::string frame = "";
  std::string outputName = "";
};

geometry_msgs::PoseStamped transformToPose(const geometry_msgs::TransformStamped& transform) {
  geometry_msgs::PoseStamped pose;

  // Copy the header
  pose.header = transform.header;

  // Set the position
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;

  // Set the orientation
  pose.pose.orientation = transform.transform.rotation;

  return pose;
}

Eigen::Matrix4d transformToMatrix(const geometry_msgs::TransformStamped& transform) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

  // Extract translation
  matrix(0, 3) = transform.transform.translation.x;
  matrix(1, 3) = transform.transform.translation.y;
  matrix(2, 3) = transform.transform.translation.z;

  // Extract rotation as a quaternion
  Eigen::Quaterniond quaternion(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y,
                                transform.transform.rotation.z);

  // Convert quaternion to a rotation matrix
  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

  // Set the rotation matrix in the top-left 3x3 part of the transformation matrix
  matrix.block<3, 3>(0, 0) = rotation_matrix;

  return matrix;
}

geometry_msgs::PoseStamped matrixToPose(const Eigen::Matrix4d& transform_matrix, const std::string& frame_id, const ros::Time& stamp) {
  geometry_msgs::PoseStamped pose;

  // Set the header
  pose.header.frame_id = frame_id;
  pose.header.stamp = stamp;

  // Extract translation
  pose.pose.position.x = transform_matrix(0, 3);
  pose.pose.position.y = transform_matrix(1, 3);
  pose.pose.position.z = transform_matrix(2, 3);

  // Extract rotation
  Eigen::Matrix3d rotation_matrix = transform_matrix.block<3, 3>(0, 0);
  Eigen::Quaterniond quaternion(rotation_matrix);

  pose.pose.orientation.x = quaternion.x();
  pose.pose.orientation.y = quaternion.y();
  pose.pose.orientation.z = quaternion.z();
  pose.pose.orientation.w = quaternion.w();

  return pose;
}

Eigen::Matrix4d poseToMatrix(const geometry_msgs::PoseStamped& pose) {
  Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();

  // Extract position (translation)
  transform_matrix(0, 3) = pose.pose.position.x;
  transform_matrix(1, 3) = pose.pose.position.y;
  transform_matrix(2, 3) = pose.pose.position.z;

  // Extract orientation (rotation) as quaternion
  Eigen::Quaterniond quaternion(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);

  // Convert quaternion to rotation matrix
  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

  // Set the rotation matrix in the top-left 3x3 part of the transformation matrix
  transform_matrix.block<3, 3>(0, 0) = rotation_matrix;

  return transform_matrix;
}

geometry_msgs::TransformStamped invertTransform(const geometry_msgs::TransformStamped& transform) {
  geometry_msgs::TransformStamped inverted_transform;

  // Swap the frames
  inverted_transform.header.stamp = transform.header.stamp;
  inverted_transform.header.frame_id = transform.child_frame_id;
  inverted_transform.child_frame_id = transform.header.frame_id;

  // Convert geometry_msgs::Transform to tf2::Transform
  tf2::Transform tf2_transform;
  tf2::fromMsg(transform.transform, tf2_transform);

  // Invert the transform
  tf2::Transform tf2_inverted = tf2_transform.inverse();

  // Convert back to geometry_msgs::Transform
  inverted_transform.transform = tf2::toMsg(tf2_inverted);

  return inverted_transform;
}

std::vector<PrepPrameters> parseBagTopicPairs(const XmlRpc::XmlRpcValue& param) {
  std::vector<PrepPrameters> bagTopicPairs;

  // Validate the parameter is an array
  if (param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("Parameter 'bag_topic_pairs' must be an array.");
    return bagTopicPairs;
  }

  // Iterate over the array
  for (int i = 0; i < param.size(); ++i) {
    if (param[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR("Each entry in 'bag_topic_pairs' must be a struct.");
      continue;
    }

    // Extract bag, topic, and frame
    PrepPrameters quartet;
    if (param[i].hasMember("bag") && param[i]["bag"].getType() == XmlRpc::XmlRpcValue::TypeString) {
      quartet.bag = static_cast<std::string>(param[i]["bag"]);
    } else {
      ROS_ERROR("Missing or invalid 'bag' in entry %d.", i);
      continue;
    }

    if (param[i].hasMember("topic") && param[i]["topic"].getType() == XmlRpc::XmlRpcValue::TypeString) {
      quartet.topic = static_cast<std::string>(param[i]["topic"]);
    } else {
      ROS_ERROR("Missing or invalid 'topic' in entry %d.", i);
      continue;
    }

    if (param[i].hasMember("frame") && param[i]["frame"].getType() == XmlRpc::XmlRpcValue::TypeString) {
      quartet.frame = static_cast<std::string>(param[i]["frame"]);
    } else {
      ROS_ERROR("Missing or invalid 'frame' in entry %d.", i);
      continue;
    }

    if (param[i].hasMember("test_name") && param[i]["test_name"].getType() == XmlRpc::XmlRpcValue::TypeString) {
      quartet.outputName = static_cast<std::string>(param[i]["test_name"]);
    } else {
      ROS_ERROR("Missing or invalid 'test_name' in entry %d.", i);
      continue;
    }

    bagTopicPairs.push_back(quartet);
  }

  return bagTopicPairs;
}

void parseRosbagToTum(const std::string& bagPath, const std::string& topicName, const std::string& outputPath, tf2_ros::Buffer& tf_buffer,
                      const std::string& targetFrame, const std::string& sourceFrame, bool enable_interpolation, rosbag::Bag& outputBag) {
  rosbag::Bag bag;
  try {
    bag.open(bagPath, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& e) {
    ROS_ERROR_STREAM("Failed to open bag file: " << e.what());
    return;
  }

  const std::string poseLogFileHeader_ = "# timestamp x y z q_x q_y q_z q_w";
  std::ofstream tumFile;

  // Open file and set numerical precision to the max.
  tumFile.open(outputPath, std::ios_base::app);
  tumFile.precision(std::numeric_limits<double>::max_digits10);
  tumFile << poseLogFileHeader_ << std::endl;

  //# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  const std::string covarianceFileHeader = "timestamp x y z roll pitch yaw";
  std::ofstream possibleCovarianceFile;

  std::string covOutputPath = outputPath.substr(0, outputPath.size() - 4) + "_covariance.tum";

  possibleCovarianceFile.open(covOutputPath, std::ios_base::app);
  possibleCovarianceFile.precision(std::numeric_limits<double>::max_digits10);
  possibleCovarianceFile << covarianceFileHeader << std::endl;

  if (!possibleCovarianceFile.is_open()) {
    ROS_ERROR_STREAM("Failed to open possibleCovarianceFile: " << covOutputPath);
    return;
  }

  if (!tumFile.is_open()) {
    ROS_ERROR_STREAM("Failed to open output file: " << outputPath);
    return;
  }

  rosbag::View view(bag, rosbag::TopicQuery(topicName));
  if (view.size() == 0) {
    std::string errorMsg = "No messages found for topic: " + topicName + " in bag: " + bagPath;
    ROS_ERROR_STREAM(errorMsg);
    bag.close();
    tumFile.close();
    possibleCovarianceFile.close();
    throw std::runtime_error(errorMsg);
  }
  for (const rosbag::MessageInstance& msg : view) {
    if (msg.getDataType() == "nav_msgs/Odometry") {
      nav_msgs::Odometry::ConstPtr odometry = msg.instantiate<nav_msgs::Odometry>();

      if (odometry) {
        if (odometry->child_frame_id != sourceFrame) {
          ROS_ERROR_STREAM("Child frame id does not match the source frame id");
          return;
        }
        geometry_msgs::PoseStamped transformed_pose;
        try {
          if (!tf_buffer.canTransform(sourceFrame, targetFrame, odometry->header.stamp, ros::Duration(0))) {
            ROS_ERROR_STREAM("Failed to find transform from " << sourceFrame << " to " << targetFrame);
            continue;
          }

          if (enable_interpolation) {
            ROS_ERROR_STREAM("Interpolation feature is currently removed.");
            throw std::runtime_error("Unsupported feature");
            // geometry_msgs::TransformStamped transform =
            //     tf_buffer.lookupTransform(sourceFrame, targetFrame, odometry->header.stamp, ros::Duration(0));
            // transform.header.frame_id = odometry->header.frame_id;
            // transform.header.stamp = odometry->header.stamp;
            // transformed_pose = transformToPose(transform);

          } else {
            geometry_msgs::TransformStamped transform =
                tf_buffer.lookupTransform(sourceFrame, targetFrame, odometry->header.stamp, ros::Duration(0));

            // Convert odometry pose to PoseStamped
            geometry_msgs::PoseStamped pose;
            pose.header = odometry->header;
            pose.pose = odometry->pose.pose;

            Eigen::Matrix4d poseAsMat = poseToMatrix(pose);
            Eigen::Matrix4d transformAsMat = transformToMatrix(transform);
            Eigen::Matrix4d transformedPoseAsMat = poseAsMat * transformAsMat;
            transformed_pose = matrixToPose(transformedPoseAsMat, targetFrame, odometry->header.stamp);
          }
          nav_msgs::Odometry asOdom;
          asOdom.header = odometry->header;
          asOdom.child_frame_id = targetFrame;
          asOdom.pose.pose = transformed_pose.pose;

          outputBag.write(topicName, odometry->header.stamp, asOdom);

          double timestamp = odometry->header.stamp.toSec();
          const auto& position = transformed_pose.pose.position;
          const auto& orientation = transformed_pose.pose.orientation;

          tumFile << timestamp << " " << position.x << " " << position.y << " " << position.z << " " << orientation.x << " "
                  << orientation.y << " " << orientation.z << " " << orientation.w << std::endl;

        } catch (const tf2::TransformException& ex) {
          ROS_ERROR_STREAM("Failed to transform pose: " << ex.what());
          bag.close();
          tumFile.close();
          possibleCovarianceFile.close();
          return;
        }
      }
    } else if (msg.getDataType() == "geometry_msgs/PoseStamped") {
      geometry_msgs::PoseStamped::ConstPtr pose = msg.instantiate<geometry_msgs::PoseStamped>();
      if (pose) {
        geometry_msgs::PoseStamped transformed_pose;
        try {
          if (!tf_buffer.canTransform(sourceFrame, targetFrame, pose->header.stamp, ros::Duration(0))) {
            ROS_ERROR_STREAM("Failed to find transform from " << sourceFrame << " to " << targetFrame);
            continue;
          }

          if (enable_interpolation) {
            ROS_ERROR_STREAM("Interpolation feature is currently removed.");
            throw std::runtime_error("Unsupported feature");
            // geometry_msgs::TransformStamped transform =
            //     tf_buffer.lookupTransform(sourceFrame, targetFrame, pose->header.stamp, ros::Duration(0));
            // transform.header.frame_id = pose->header.frame_id;
            // transform.header.stamp = pose->header.stamp;
            // transformed_pose = transformToPose(transform);

          } else {
            geometry_msgs::TransformStamped transform =
                tf_buffer.lookupTransform(sourceFrame, targetFrame, pose->header.stamp, ros::Duration(0));

            Eigen::Matrix4d poseAsMat = poseToMatrix(*pose);
            Eigen::Matrix4d transformAsMat = transformToMatrix(transform);
            Eigen::Matrix4d transformedPoseAsMat = poseAsMat * transformAsMat;
            transformed_pose = matrixToPose(transformedPoseAsMat, targetFrame, pose->header.stamp);
          }

          nav_msgs::Odometry asOdom;
          asOdom.header = pose->header;
          asOdom.child_frame_id = targetFrame;
          asOdom.pose.pose = transformed_pose.pose;

          outputBag.write(topicName, pose->header.stamp, asOdom);

          double timestamp = pose->header.stamp.toSec();
          const auto& position = transformed_pose.pose.position;
          const auto& orientation = transformed_pose.pose.orientation;

          tumFile << timestamp << " " << position.x << " " << position.y << " " << position.z << " " << orientation.x << " "
                  << orientation.y << " " << orientation.z << " " << orientation.w << std::endl;
        } catch (const tf2::TransformException& ex) {
          ROS_ERROR_STREAM("Failed to transform pose: " << ex.what());
          bag.close();
          tumFile.close();
          possibleCovarianceFile.close();
          return;
        }
      }
    } else if (msg.getDataType() == "geometry_msgs/PoseWithCovarianceStamped") {
      geometry_msgs::PoseWithCovarianceStamped::ConstPtr poseCov = msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
      if (poseCov) {
        geometry_msgs::PoseStamped transformed_pose;
        try {
          if (!tf_buffer.canTransform(sourceFrame, targetFrame, poseCov->header.stamp, ros::Duration(0))) {
            ROS_ERROR_STREAM("Failed to find transform from " << sourceFrame << " to " << targetFrame);
            continue;
          }

          if (enable_interpolation) {
            ROS_ERROR_STREAM("Interpolation feature is currently removed. Please disable it.");
            throw std::runtime_error("Unsupported feature");

            // geometry_msgs::TransformStamped transform =
            //     tf_buffer.lookupTransform(sourceFrame, targetFrame, poseCov->header.stamp, ros::Duration(0));
            // transform.header.frame_id = sourceFrame;
            // transform.header.stamp = poseCov->header.stamp;
            // transformed_pose = transformToPose(transform);

          } else {
            geometry_msgs::TransformStamped transform =
                tf_buffer.lookupTransform(sourceFrame, targetFrame, poseCov->header.stamp, ros::Duration(0));

            // Convert poseCov pose to PoseStamped
            geometry_msgs::PoseStamped pose;
            pose.header = poseCov->header;
            pose.pose = poseCov->pose.pose;

            Eigen::Matrix4d poseAsMat = poseToMatrix(pose);
            Eigen::Matrix4d transformAsMat = transformToMatrix(transform);
            Eigen::Matrix4d transformedPoseAsMat = poseAsMat * transformAsMat;
            transformed_pose = matrixToPose(transformedPoseAsMat, targetFrame, poseCov->header.stamp);
          }

          nav_msgs::Odometry asOdom;
          asOdom.header = poseCov->header;
          asOdom.child_frame_id = targetFrame;
          asOdom.pose.pose = transformed_pose.pose;

          outputBag.write(topicName, poseCov->header.stamp, asOdom);

          double timestamp = poseCov->header.stamp.toSec();
          const auto& position = transformed_pose.pose.position;
          const auto& orientation = transformed_pose.pose.orientation;

          tumFile << timestamp << " " << position.x << " " << position.y << " " << position.z << " " << orientation.x << " "
                  << orientation.y << " " << orientation.z << " " << orientation.w << std::endl;

          auto estGnssOfflinePoseMeasUnaryNoiseList = poseCov->pose.covariance;
          Eigen::Matrix<double, 6, 1> estGnssOfflinePoseMeasUnaryNoise;
          estGnssOfflinePoseMeasUnaryNoise << sqrt(estGnssOfflinePoseMeasUnaryNoiseList[0]), sqrt(estGnssOfflinePoseMeasUnaryNoiseList[7]),
              sqrt(estGnssOfflinePoseMeasUnaryNoiseList[14]), sqrt(estGnssOfflinePoseMeasUnaryNoiseList[21]),
              sqrt(estGnssOfflinePoseMeasUnaryNoiseList[28]), sqrt(estGnssOfflinePoseMeasUnaryNoiseList[35]);

          possibleCovarianceFile << timestamp;
          for (int i = 0; i < estGnssOfflinePoseMeasUnaryNoise.size(); ++i) {
            possibleCovarianceFile << " " << estGnssOfflinePoseMeasUnaryNoise[i];
          }
          possibleCovarianceFile << std::endl;

        } catch (const tf2::TransformException& ex) {
          ROS_ERROR_STREAM("Failed to transform pose: " << ex.what());
          bag.close();
          tumFile.close();
          possibleCovarianceFile.close();
          return;
        }
      }

    } else if (msg.getDataType() == "geometry_msgs/PointStamped") {
      geometry_msgs::PointStamped::ConstPtr point = msg.instantiate<geometry_msgs::PointStamped>();

      if (point) {
        geometry_msgs::PoseStamped transformed_pose;
        try {
          if (sourceFrame != targetFrame) {
            ROS_ERROR_STREAM_THROTTLE(1.0, "You are trying to transform a position mesaurement. This is not well handled."
                                               << sourceFrame << " to " << targetFrame);
          }

          if (!tf_buffer.canTransform(sourceFrame, targetFrame, point->header.stamp, ros::Duration(0))) {
            ROS_ERROR_STREAM("Failed to find transform from " << sourceFrame << " to " << targetFrame);
            continue;
          }

          if (enable_interpolation) {
            ROS_ERROR_STREAM("Interpolation feature is currently removed. Please disable it.");
            throw std::runtime_error("Unsupported feature");

            // geometry_msgs::TransformStamped transform =
            //     tf_buffer.lookupTransform(sourceFrame, targetFrame, point->header.stamp, ros::Duration(0));
            // transform.header.frame_id = sourceFrame;
            // transform.header.stamp = point->header.stamp;
            // transformed_pose = transformToPose(transform);

          } else {
            geometry_msgs::TransformStamped transform =
                tf_buffer.lookupTransform(sourceFrame, targetFrame, point->header.stamp, ros::Duration(0));

            // Convert point pose to PoseStamped
            geometry_msgs::PoseStamped pose;
            pose.header = point->header;
            pose.pose.position = point->point;

            // Set the orientation to identity (no rotation)
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            Eigen::Matrix4d poseAsMat = poseToMatrix(pose);
            Eigen::Matrix4d transformAsMat = transformToMatrix(transform);
            Eigen::Matrix4d transformedPoseAsMat = poseAsMat * transformAsMat;
            transformed_pose = matrixToPose(transformedPoseAsMat, targetFrame, point->header.stamp);
          }

          double timestamp = point->header.stamp.toSec();
          const auto& position = transformed_pose.pose.position;
          const auto& orientation = transformed_pose.pose.orientation;

          tumFile << timestamp << " " << position.x << " " << position.y << " " << position.z << " " << orientation.x << " "
                  << orientation.y << " " << orientation.z << " " << orientation.w << std::endl;

        } catch (const tf2::TransformException& ex) {
          ROS_ERROR_STREAM("Failed to transform pose: " << ex.what());
          bag.close();
          tumFile.close();
          possibleCovarianceFile.close();
          return;
        }
      }

    } else {
      ROS_ERROR_STREAM("NOT SUPPORTED TYPE");
      bag.close();
      tumFile.close();
      possibleCovarianceFile.close();
      return;
    }
  }

  bag.close();
  tumFile.close();
  possibleCovarianceFile.close();
}

void processTF(const std::vector<std::string>& tfContainingBags, tf2_ros::Buffer& tf_buffer) {
  ROS_INFO("Reading a rosbag file to get the duration of the tf buffer");

  if (tfContainingBags.empty()) {
    ROS_ERROR("No TF-containing bags provided.");
    return;
  }

  std::vector<rosbag::Bag> bags;
  bags.reserve(tfContainingBags.size());

  ROS_INFO_STREAM("Number of Bags for TF: " << tfContainingBags.size());

  for (const auto& path : tfContainingBags) {
    ROS_INFO_STREAM("Opening bag: " << path);
    bags.emplace_back();
    bags.back().open(path, rosbag::bagmode::Read);
  }

  ROS_INFO_STREAM("Combining TF for offline reading.");
  std::vector<std::string> tf_topics{"/tf_static"};  //"/tf"
  rosbag::View combined_view;
  for (auto& tfbag : bags) {
    combined_view.addQuery(tfbag, rosbag::TopicQuery(tf_topics));
  }

  if (combined_view.size() == 0) {
    ROS_WARN("The combined view contains no messages. Ensure the bags have relevant TF topics.");
    return;
  }

  // bool got_tf_static = false;

  for (const rosbag::MessageInstance& m : combined_view) {
    const std::string& topic_name = m.getTopic();

    if (topic_name == "/tf") {
      tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
      if (tf_msg) {
        for (const geometry_msgs::TransformStamped& transform : tf_msg->transforms) {
          tf_buffer.setTransform(transform, "default_authority", false);
        }
      }
    } else if (topic_name == "/tf_static") {
      // if (!got_tf_static) {
      tf2_msgs::TFMessage::ConstPtr tf_static_msg = m.instantiate<tf2_msgs::TFMessage>();
      if (tf_static_msg) {
        for (const geometry_msgs::TransformStamped& transform : tf_static_msg->transforms) {
          tf_buffer.setTransform(transform, "default_authority", true);
        }
        // ROS_INFO_STREAM("Got tf_static transforms.");
        // got_tf_static = true;
      } else {
        ROS_ERROR_STREAM("Failed to get tf_static message.");
        throw std::runtime_error("Failed to instantiate tf_static");
        return;
      }
      // }
    }
  }
}

// Main boi
int main(int argc, char** argv) {
  ros::init(argc, argv, "evo_converter");
  ros::NodeHandle nh("~");

  std::string bag_file_directory = "";
  if (!nh.getParam("bag_file_directory", bag_file_directory)) {
    ROS_ERROR("Parameter 'bag_file_directory' not set!");
    return -1;
  }

  std::string missionPrefix = "";
  if (!nh.getParam("prefix", missionPrefix)) {
    ROS_ERROR("Parameter 'missionPrefix' not set!");
    return -1;
  }

  // Ensure bag_file_directory has a trailing slash
  if (!bag_file_directory.empty() && bag_file_directory.back() != '/') {
    bag_file_directory += '/';
  }

  // std::string enable_interpolation_str = "";
  bool enable_interpolation = false;
  if (!nh.getParam("enable_interpolation", enable_interpolation)) {
    ROS_ERROR("Parameter 'enable_interpolation' not set!");
    return -1;
  }

  std::string outputDir = "GrandTourBestTour";
  if (!nh.getParam("output_folder_path", outputDir)) {
    ROS_ERROR("Parameter 'output_folder_path' not set!");
    return -1;
  }

  std::string targetFrame = "GrandTourBestTour";
  if (!nh.getParam("/target_frame", targetFrame)) {
    ROS_ERROR("Parameter 'target_frame' not set!");
    return -1;
  }

  XmlRpc::XmlRpcValue param;
  if (!nh.getParam("/bag_topic_pairs", param)) {
    ROS_ERROR("Failed to retrieve 'bag_topic_pairs' parameter.");
    return 1;
  }

  std::vector<PrepPrameters> bagTopicPairs = parseBagTopicPairs(param);

  // Output the parsed results for verification
  for (auto& quartet : bagTopicPairs) {
    if ((!missionPrefix.empty()) && (quartet.bag[0] == '_')) {
      quartet.bag = missionPrefix + quartet.bag;
    }

    if (quartet.frame.empty()) {
      ROS_INFO_STREAM("\033[1;33mOnly tf Bag: " << quartet.bag << "\033[0m");
    } else {
      ROS_INFO_STREAM("Bag: " << quartet.bag << ", Topic: " << quartet.topic << ", Frame: " << quartet.frame
                              << ", Output Name: " << quartet.outputName);
    }
  }

  if (!std::filesystem::exists(outputDir)) {
    ROS_ERROR_STREAM("Output directory does not exist: " << outputDir);
    std::filesystem::create_directories(outputDir);
  }

  for (const auto& entry : std::filesystem::directory_iterator(outputDir)) {
    if (entry.path().extension() == ".tum") {
      std::filesystem::remove(entry.path());
      ROS_INFO_STREAM("\033[1;31mDeleted file: " << entry.path() << "\033[0m");
    }
  }

  std::vector<std::string> tfContainingBags;

  for (const auto& quartet : bagTopicPairs) {
    tfContainingBags.push_back(bag_file_directory + quartet.bag);
  }

  if (tfContainingBags.empty()) {
    ROS_ERROR("No TF-containing bags provided.");
    return -1;
  }

  rosbag::Bag exampleBag(tfContainingBags.front(), rosbag::bagmode::Read);
  rosbag::View full_view(exampleBag);
  ros::Time start_time = full_view.getBeginTime();
  ros::Time end_time = full_view.getEndTime();

  if (start_time == ros::TIME_MIN || end_time == ros::TIME_MIN) {
    ROS_ERROR("Start time or end time is invalid (ros::TIME_MIN).");
    return -1;
  }

  if (start_time == ros::TIME_MAX || end_time == ros::TIME_MAX) {
    ROS_ERROR("Start time or end time is invalid (ros::TIME_MAX).");
    return -1;
  }
  ros::Duration bag_duration = end_time - start_time;

  tf2_ros::Buffer tf_buffer(bag_duration);
  tf2_ros::TransformListener tf_listener(tf_buffer);

  processTF(tfContainingBags, tf_buffer);

  ROS_INFO_STREAM("\033[1;32mTF processing has been completed successfully.\033[0m");

  rosbag::Bag outputBag;
  outputBag.setCompression(rosbag::compression::LZ4);
  std::string outputBagPath = outputDir + "/" + missionPrefix + "_evaluation_pose_collections.bag";
  try {
    outputBag.open(outputBagPath, rosbag::bagmode::Write);
  } catch (const rosbag::BagException& e) {
    ROS_ERROR_STREAM("Failed to open bag file: " << e.what());
    return -1;
  }

  for (const auto& quartet : bagTopicPairs) {
    std::string bagPath = bag_file_directory + quartet.bag;
    std::string topicName = quartet.topic;
    std::string sourceFrame = quartet.frame;
    std::string outputName = quartet.outputName;

    if (topicName.empty()) {
      continue;
    }

    ROS_INFO_STREAM("Bag Path: " << bagPath);
    ROS_INFO_STREAM("Topic Name: " << topicName);
    ROS_INFO_STREAM("Source Frame: " << sourceFrame);
    ROS_INFO_STREAM("Test Name: " << outputName);

    std::string bagname = outputName;
    if (outputName.empty()) {
      bagname = quartet.bag.substr(0, quartet.bag.size() - 4);
    }

    std::string outputPath = (std::filesystem::path(outputDir) / (bagname + ".tum")).string();

    ROS_INFO_STREAM("\033[1;32mProcessing: " << bagPath << " (Topic: " << topicName << ") -> " << outputPath << ".\033[0m");

    parseRosbagToTum(bagPath, topicName, outputPath, tf_buffer, targetFrame, sourceFrame, enable_interpolation, outputBag);
  }
  outputBag.close();
  ROS_INFO("Processing completed.");
  ros::Duration(1.0).sleep();
  ROS_INFO_STREAM("\033[1;32mProcessing completed. Self-terminating.\033[0m");
  return 0;
}

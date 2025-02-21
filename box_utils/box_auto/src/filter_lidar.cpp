#include <XmlRpcValue.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

std::vector<size_t> getInverseIndices(const std::vector<size_t>& selected_indices, size_t total_points) {
  std::unordered_set<size_t> selected_set(selected_indices.begin(), selected_indices.end());
  std::vector<size_t> inverse;
  inverse.reserve(total_points - selected_indices.size());
  for (size_t i = 0; i < total_points; ++i) {
    if (selected_set.find(i) == selected_set.end()) {
      inverse.push_back(static_cast<size_t>(i));
    }
  }
  return inverse;
}

/**
 * @brief Calculate angle (degrees) of (x,y) around Z in [0..360).
 * @param x  X coordinate
 * @param y  Y coordinate
 */
float calculateAngleAroundZ(float x, float y) {
  float angle_deg = std::atan2(y, x) * 180.0f / static_cast<float>(M_PI);
  return (angle_deg >= 0.0f) ? angle_deg : (angle_deg + 360.0f);
}

/// @brief check if angle is in any of the ranges
/// @param angle
/// @param ranges
/// @return
bool isAngleInRange(float angle, const std::vector<std::pair<float, float>>& ranges) {
  for (const auto& range : ranges) {
    if (angle >= range.first && angle <= range.second) {
      return true;
    }
  }
  return false;
}

bool rosToOpen3d(const sensor_msgs::PointCloud2& cloud, open3d::geometry::PointCloud& o3d_pc) {
  const auto ros_pc2 = &cloud;
  const uint32_t num_points = ros_pc2->height * ros_pc2->width;

  if (ros_pc2->fields.size() < 3) {
    // Early exit
    // Crushes if there are no points
    std::cout << "ros_pc2->fields.size(): " << ros_pc2->fields.size() << " Exitting." << std::endl;
    return false;
  }

  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
  o3d_pc.points_.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
    o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
  }

  return true;
}

/**
 * @brief Print basic point field info for a sensor_msgs::PointCloud2.
 */
void printPointCloud2Fields(const sensor_msgs::PointCloud2& cloud_msg) {
  ROS_INFO_STREAM("Frame: " << cloud_msg.header.frame_id << ", stamp: " << cloud_msg.header.stamp << ", height: " << cloud_msg.height
                            << ", width: " << cloud_msg.width);
  ROS_INFO_STREAM("Fields:");
  for (size_t i = 0; i < cloud_msg.fields.size(); ++i) {
    const auto& f = cloud_msg.fields[i];
    ROS_INFO_STREAM("  - name=" << f.name << ", offset=" << f.offset << ", datatype=" << static_cast<int>(f.datatype)
                                << ", count=" << f.count);
  }
  ROS_INFO_STREAM("Point step: " << cloud_msg.point_step << ", row step: " << cloud_msg.row_step);
}

/**
 * @brief Copy only selected point indices from an input PointCloud2 into a new PointCloud2,
 *        preserving all fields exactly.
 *
 * @param out_msg  Filtered output PointCloud2
 * @param in_msg  The original unfiltered PointCloud2
 * @param indices A list of point indices to keep
 * @return A new sensor_msgs::PointCloud2 containing only the selected points
 */
sensor_msgs::PointCloud2& copyPointsByIndex(sensor_msgs::PointCloud2& out_msg, const sensor_msgs::PointCloud2& in_msg,
                                            const std::vector<int>& indices) {
  // Copy metadata
  out_msg.header = in_msg.header;
  out_msg.fields = in_msg.fields;
  out_msg.is_bigendian = in_msg.is_bigendian;
  out_msg.point_step = in_msg.point_step;
  out_msg.is_dense = false;  // Assume some points may have been removed

  // Set dimensions for an unorganized point cloud
  out_msg.height = 1;
  out_msg.width = static_cast<uint32_t>(indices.size());
  out_msg.row_step = out_msg.width * out_msg.point_step;
  out_msg.data.resize(out_msg.row_step);

  const size_t num_input_points = static_cast<size_t>(in_msg.width * in_msg.height);
  const uint8_t* input_data_ptr = in_msg.data.data();
  uint8_t* output_data_ptr = out_msg.data.data();

  // Iterate through indices and copy corresponding points
  for (size_t out_i = 0; out_i < indices.size(); ++out_i) {
    int in_idx = indices[out_i];
    if (in_idx < 0 || static_cast<size_t>(in_idx) >= num_input_points) {
      continue;
    }
    const uint8_t* src_ptr = input_data_ptr + in_idx * in_msg.point_step;
    uint8_t* dst_ptr = output_data_ptr + out_i * out_msg.point_step;
    std::copy(src_ptr, src_ptr + in_msg.point_step, dst_ptr);
  }

  return out_msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_filter");
  ros::NodeHandle nh("~");

  bool isGlobalPathAvailable = false;

  std::string input_bag_path = "";
  std::string output_bag_path = "";
  std::string global_input_bag_path = "";
  std::string global_output_bag_path = "";
  nh.getParam("global_input_bag_path", global_input_bag_path);
  nh.getParam("global_output_bag_path", global_output_bag_path);

  bool slow_down_for_debug = false;
  nh.getParam("slow_down_for_debug", slow_down_for_debug);

  if (!global_input_bag_path.empty()) {
    isGlobalPathAvailable = true;
    input_bag_path = global_input_bag_path;
  }

  if (!global_output_bag_path.empty()) {
    isGlobalPathAvailable = true;
    output_bag_path = global_output_bag_path;
  }

  if (!isGlobalPathAvailable) {
    // Get bag paths from rosparam
    std::string base_path = "/tmp/";
    nh.getParam("base_path", base_path);

    // Ensure base_path ends with a trailing '/'
    if (!base_path.empty() && base_path.back() != '/') {
      base_path += '/';
    }

    std::string input_bag_name = "input.bag";
    std::string output_bag_name = "output.bag";
    nh.getParam("input_bag_name", input_bag_name);
    nh.getParam("output_bag_name", output_bag_name);

    // Ensure input and output bag names end with ".bag"
    if (input_bag_name.size() < 4 || input_bag_name.substr(input_bag_name.size() - 4) != ".bag") {
      input_bag_name += ".bag";
    }
    if (output_bag_name.size() < 4 || output_bag_name.substr(output_bag_name.size() - 4) != ".bag") {
      output_bag_name += ".bag";
    }

    // Check if base path exists, if not create the directory
    if (!std::filesystem::exists(base_path)) {
      try {
        std::filesystem::create_directories(base_path);
      } catch (const std::filesystem::filesystem_error& e) {
        ROS_ERROR("Failed to create base directory: %s, error: %s", base_path.c_str(), e.what());
        return -1;
      }
    }

    input_bag_path = base_path + input_bag_name;
    output_bag_path = base_path + output_bag_name;
  }

  // Check if input bag exists
  if (!std::filesystem::exists(input_bag_path)) {
    ROS_ERROR("Input bag file does not exist: %s", input_bag_path.c_str());
    return -1;
  }

  // Check if input and output bag paths are the same
  if (input_bag_path == output_bag_path) {
    ROS_ERROR("Input and output bag paths cannot be the same: %s", input_bag_path.c_str());
    return -1;
  }

  // Remove output bag if it exists
  if (std::filesystem::exists(output_bag_path)) {
    try {
      std::filesystem::remove(output_bag_path);
    } catch (const std::filesystem::filesystem_error& e) {
      ROS_ERROR("Failed to remove existing output bag file: %s, error: %s", output_bag_path.c_str(), e.what());
      return -1;
    }
  }

  // Operated pointcloud topic
  std::string input_topic = "";

  // Radius based density filter
  double radius_search = 0.0;
  int min_neighbors = 0;
  bool disable_density_filtering = false;
  double angular_filtering_range = 1.0;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("bounding_box_marker", 1, true);

  // Crop box
  std::vector<float> crop_min, crop_max;

  // Get Mode ("livox" or "hesai")
  std::string operation_mode = "";

  if (input_bag_path.find("livox") != std::string::npos) {
    operation_mode = "livox";
  } else if (input_bag_path.find("hesai") != std::string::npos) {
    operation_mode = "hesai";
  } else {
    ROS_ERROR("Input bag path does not specify a valid operation mode (livox or hesai): %s", input_bag_path.c_str());
    return -1;
  }

  ROS_INFO("Current mode: %s", operation_mode.c_str());

  std::string mode_ns = "/mode/" + operation_mode;

  std::vector<std::string> input_topics;
  // Load angle ranges from parameters

  if (!nh.getParam(mode_ns + "/input_topic", input_topics)) {
    ROS_ERROR("Failed to load input topics for mode: %s", operation_mode.c_str());
    return -1;
  }

  if (!nh.getParam(mode_ns + "/angular_filtering_range", angular_filtering_range)) {
    ROS_ERROR("Failed to load angular_filtering_range for mode: %s", operation_mode.c_str());
    return -1;
  }

  if (!nh.getParam(mode_ns + "/disable_density_filtering", disable_density_filtering)) {
    ROS_ERROR("Failed to load disable_density_filtering for mode: %s", operation_mode.c_str());
    return -1;
  }

  if (!nh.getParam(mode_ns + "/density_filtering_radius", radius_search)) {
    ROS_ERROR("Failed to load density filtering radius for mode: %s", operation_mode.c_str());
    return -1;
  }
  if (!nh.getParam(mode_ns + "/density_filtering_min_neighbors", min_neighbors)) {
    ROS_ERROR("Failed to load density filtering min neighbors for mode: %s", operation_mode.c_str());
    return -1;
  }
  if (!nh.getParam(mode_ns + "/crop_box/min", crop_min)) {
    ROS_ERROR("Failed to load crop box min for mode: %s", operation_mode.c_str());
    return -1;
  }
  if (!nh.getParam(mode_ns + "/crop_box/max", crop_max)) {
    ROS_ERROR("Failed to load crop box max for mode: %s", operation_mode.c_str());
    return -1;
  }

  ROS_INFO("Crop Box Min: [%f, %f, %f, %f]", crop_min[0], crop_min[1], crop_min[2], crop_min[3]);
  ROS_INFO("Crop Box Max: [%f, %f, %f, %f]", crop_max[0], crop_max[1], crop_max[2], crop_max[3]);

  // Load angle ranges using XmlRpcValue
  XmlRpc::XmlRpcValue angle_ranges_raw;
  if (!nh.getParam(mode_ns + "/angle_ranges", angle_ranges_raw)) {
    ROS_ERROR("Failed to load angle ranges for mode: %s", operation_mode.c_str());
    return -1;
  }

  // Validate and convert the raw data
  std::vector<std::pair<float, float>> angle_ranges;
  if (angle_ranges_raw.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < angle_ranges_raw.size(); ++i) {
      if (angle_ranges_raw[i].getType() == XmlRpc::XmlRpcValue::TypeArray && angle_ranges_raw[i].size() == 2 &&
          angle_ranges_raw[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
          angle_ranges_raw[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        angle_ranges.emplace_back(static_cast<double>(angle_ranges_raw[i][0]), static_cast<double>(angle_ranges_raw[i][1]));
      } else {
        ROS_WARN("Invalid angle range at index %d; it must be an array of two floats.", i);
      }
    }
  } else {
    ROS_ERROR("Angle ranges parameter must be an array of arrays.");
    return -1;
  }

  ROS_INFO("Loaded angle ranges for mode: %s", operation_mode.c_str());
  for (const auto& range : angle_ranges) {
    ROS_INFO(" - Range: [%f, %f]", range.first, range.second);
  }

  input_topic = input_topics[0];

  std::string output_topic = "";
  nh.getParam("output_topic", output_topic);

  // If output topic is empty, use input topic
  if (output_topic.empty()) {
    output_topic = input_topic;
  }

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic + "/filtered", 1, true);

  // Open bags
  rosbag::Bag input_bag, output_bag;

  try {
    input_bag.open(input_bag_path, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& e) {
    ROS_ERROR("Failed to open input bag: %s", e.what());
    return -1;
  }

  try {
    output_bag.open(output_bag_path, rosbag::bagmode::Write);
    output_bag.setCompression(rosbag::compression::LZ4);

  } catch (const rosbag::BagException& e) {
    ROS_ERROR("Failed to open output bag: %s", e.what());
    return -1;
  }

  std::string cloud_frame;
  {
    // Extract the frame id from the first valid point cloud message
    rosbag::View frame_view(input_bag, rosbag::TopicQuery(input_topic));
    for (const auto& frame_msg : frame_view) {
      sensor_msgs::PointCloud2::ConstPtr cloud_msg = frame_msg.instantiate<sensor_msgs::PointCloud2>();
      if (cloud_msg) {
        cloud_frame = cloud_msg->header.frame_id;
        break;
      }
    }
  }
  if (cloud_frame.empty()) {
    ROS_WARN("Could not determine cloud frame, defaulting to 'hesai_lidar'");
    return -1;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = cloud_frame;  // Adjust frame as needed
  marker.header.stamp = ros::Time::now();
  marker.ns = "bounding_box";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  // Compute center and size from crop_min and crop_max
  marker.pose.position.x = (crop_min[0] + crop_max[0]) / 2.0;
  marker.pose.position.y = (crop_min[1] + crop_max[1]) / 2.0;
  marker.pose.position.z = (crop_min[2] + crop_max[2]) / 2.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = std::fabs(crop_max[0] - crop_min[0]);
  marker.scale.y = std::fabs(crop_max[1] - crop_min[1]);
  marker.scale.z = std::fabs(crop_max[2] - crop_min[2]);

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;

  marker.lifetime = ros::Duration();  // marker persists indefinitely

  marker_pub.publish(marker);

  // Lets iterate the bag
  rosbag::View view(input_bag);

  // Position of the livox tag field in the point cloud
  int tag_offset = -1;

  Eigen::Vector3d min_bound(crop_min[0], crop_min[1], crop_min[2]);
  Eigen::Vector3d max_bound(crop_max[0], crop_max[1], crop_max[2]);
  open3d::geometry::AxisAlignedBoundingBox aabb(min_bound, max_bound);
  ros::Rate rate(100);
  size_t total_msgs = view.size();
  size_t processed = 0;
  for (const rosbag::MessageInstance& msg : view) {
    if (msg.getTopic() == input_topic) {
      sensor_msgs::PointCloud2::ConstPtr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
      if (!cloud_msg) {
        // Not the correct type or empty
        continue;
      }

      // Read only once if livox mode is enabled.
      if (tag_offset == -1 && operation_mode == "livox") {
        for (const auto& field : cloud_msg->fields) {
          if (field.name == "tag") {
            tag_offset = field.offset;
            break;
          }
        }
        if (tag_offset == -1) {
          ROS_ERROR("Field 'tag' not found in PointCloud2 message. Are you sure Livox tag is present?");
          return -1;
        }
      }

      // Debug printing
      // printPointCloud2Fields(*cloud_msg);

      open3d::geometry::PointCloud cloudmy;

      if (!rosToOpen3d(*cloud_msg, cloudmy)) {
        std::cout << "Couldn't convert the point cloud" << std::endl;
        return -1;
      }

      std::vector<size_t> bad_indices;
      for (size_t i = 0; i < cloudmy.points_.size(); i++) {  // old index
        bool is_nan = (std::isnan(cloudmy.points_[i](0)) || std::isnan(cloudmy.points_[i](1)) || std::isnan(cloudmy.points_[i](2)));
        bool is_infinite = (std::isinf(cloudmy.points_[i](0)) || std::isinf(cloudmy.points_[i](1)) || std::isinf(cloudmy.points_[i](2)));
        if (is_nan || is_infinite) {
          bad_indices.push_back(i);
        }
      }

      if (bad_indices.size() > 0) {
        ROS_WARN_STREAM("Nan or Infinite points: " << bad_indices.size());
      }

      std::vector<size_t> box_indices_in = aabb.GetPointIndicesWithinBoundingBox(cloudmy.points_);

      // Combine bad_indices and box_indices_in into a unique set of indices
      std::unordered_set<size_t> combined_set;
      combined_set.insert(bad_indices.begin(), bad_indices.end());
      combined_set.insert(box_indices_in.begin(), box_indices_in.end());

      std::vector<size_t> combined_indices(combined_set.begin(), combined_set.end());
      std::vector<size_t> box_indices = getInverseIndices(combined_indices, cloudmy.points_.size());

      std::vector<int> next_indices;
      next_indices.reserve(box_indices.size());

      const uint8_t* data_ptr = cloud_msg->data.data();
      const size_t step = cloud_msg->point_step;
      const size_t n_xyz_pts = cloudmy.points_.size();

      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*cloud_msg, *xyz_cloud);

      for (int idx : box_indices) {
        if (idx < 0 || idx >= static_cast<int>(n_xyz_pts)) {
          continue;
        }
        // Check Livox tag
        if (tag_offset >= 0) {
          size_t offset = static_cast<size_t>(idx) * step + tag_offset;
          uint8_t tag_val = data_ptr[offset];
          if (tag_val != 0) {
            // This is how livox mid360 tags are set (0 for valid points)
            continue;
          }
        }

        // Check angle, currenty this is hardcoded instead of robot self-filter URDF based.
        const auto& pt = xyz_cloud->points[idx];
        float angle_deg = calculateAngleAroundZ(pt.x, pt.y);
        float norm = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        bool badAngle = isAngleInRange(angle_deg, angle_ranges);
        bool shouldWeRemove = badAngle && (norm < angular_filtering_range);

        // Keep the point if it is not both bad-angle and within 2 meters
        if (!shouldWeRemove) {
          next_indices.push_back(idx);
        }
      }

      std::vector<int> final_indices;
      if (disable_density_filtering) {
        // Density filtering is disabled; keep the indices from the crop box filtering stage.
        final_indices = next_indices;
      } else {
        // Create a new cloud with only the selected points (we keep track of the mapping to original indices)
        pcl::PointCloud<pcl::PointXYZ>::Ptr subset_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        subset_cloud->reserve(next_indices.size());
        std::vector<int> subset_map;
        subset_map.reserve(next_indices.size());
        for (int idx : next_indices) {
          subset_cloud->push_back(xyz_cloud->points[idx]);
          subset_map.push_back(idx);
        }

        // Apply radius outlier removal filter
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(subset_cloud);
        ror.setRadiusSearch(radius_search);
        ror.setMinNeighborsInRadius(min_neighbors);

        std::vector<int> ror_indices;
        ror.filter(ror_indices);

        final_indices.reserve(ror_indices.size());
        for (int i : ror_indices) {
          final_indices.push_back(subset_map[i]);
        }
      }

      // Get the points in the original format
      sensor_msgs::PointCloud2 filtered_msg;
      copyPointsByIndex(filtered_msg, *cloud_msg, final_indices);

      // Use header.stamp
      if (filtered_msg.header.stamp.isZero()) {
        ROS_ERROR("No valid header timestamp in filtered message, using bag time.");
        return -1;
      }

      if (pub.getNumSubscribers() > 0) {
        pub.publish(filtered_msg);
      }
      ros::spinOnce();

      // Write to output bag
      output_bag.write(output_topic, filtered_msg.header.stamp, filtered_msg);

      if (slow_down_for_debug) {
        rate.sleep();
      }

    } else {
      if (operation_mode == "livox") {
        // In grandtour Livox bag, we have IMU topics.
        sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
        if (!imu_msg) {
          continue;
        }
        output_bag.write(msg.getTopic(), imu_msg->header.stamp, imu_msg);
      } else if (operation_mode == "hesai") {
        output_bag.write(msg.getTopic(), msg.getTime(), msg);
      } else {
        ROS_ERROR("Unknown operation mode: %s", operation_mode.c_str());
        return -1;
      }
    }

    processed++;
    if ((processed % 100 == 0) || (processed == 1) || (processed == total_msgs)) {
      ROS_INFO("Processed %lu / %lu messages", (unsigned long)processed, (unsigned long)total_msgs);
    }
  }

  input_bag.close();
  output_bag.close();
  ROS_INFO("Done writing filtered bag to %s", output_bag_path.c_str());
  return 0;
}

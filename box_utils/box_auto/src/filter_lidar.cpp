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
#include <cmath>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

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
  ros::init(argc, argv, "livox_mid360_filter");
  ros::NodeHandle nh("~");

  bool isGlobalPathAvailable = false;

  std::string input_bag_path = "";
  std::string output_bag_path = "";
  std::string global_input_bag_path = "";
  std::string global_output_bag_path = "";
  nh.getParam("global_input_bag_path", global_input_bag_path);
  nh.getParam("global_output_bag_path", global_output_bag_path);

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

  // Crop box
  std::vector<float> crop_min, crop_max;

  // Get Mode ("livox" or "hesai")
  std::string operation_mode = "";

  // Angle ranges
  std::vector<std::pair<float, float>> angle_ranges;

  if (nh.getParam("operation_mode", operation_mode)) {
    ROS_INFO("Current mode: %s", operation_mode.c_str());

    std::string mode_ns = "/mode/" + operation_mode;

    std::vector<std::string> input_topics;
    // Load angle ranges from parameters

    if (!nh.getParam(mode_ns + "/input_topic", input_topics)) {
      ROS_ERROR("Failed to load input topics for mode: %s", operation_mode.c_str());
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
    // for (const auto& topic : input_topics) {
    //   input_topic = topic;
    // }

  } else {
    ROS_ERROR("Failed to read 'operation_mode' parameter");
    return -1;
  }

  std::string output_topic = "";
  nh.getParam("output_topic", output_topic);

  // If output topic is empty, use input topic
  if (output_topic.empty()) {
    output_topic = input_topic;
  }

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

  // old school PCL cropbox
  // TODO make this robot self-filter based.
  pcl::CropBox<pcl::PointXYZ> crop_box;
  crop_box.setMin(Eigen::Vector4f(crop_min[0], crop_min[1], crop_min[2], crop_min[3]));
  crop_box.setMax(Eigen::Vector4f(crop_max[0], crop_max[1], crop_max[2], crop_max[3]));
  crop_box.setNegative(true);  // keep points outside that box

  // Lets iterate the bag
  rosbag::View view(input_bag);

  // std::set<std::string> topicsInBag;
  // for (const auto& connection_info : view.getConnections()) {
  //     topicsInBag.insert(connection_info->topic);
  // }

  // Position of the livox tag field in the point cloud
  int tag_offset = -1;

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

      // Convert to a PCL XYZ cloud for bounding box, angle checks, and neighbor search
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*cloud_msg, *xyz_cloud);

      std::vector<int> box_indices;
      crop_box.setInputCloud(xyz_cloud);
      crop_box.filter(box_indices);

      std::vector<int> next_indices;
      next_indices.reserve(box_indices.size());

      const uint8_t* data_ptr = cloud_msg->data.data();
      const size_t step = cloud_msg->point_step;
      const size_t n_xyz_pts = xyz_cloud->points.size();

      for (int idx : box_indices) {
        if (idx < 0 || idx >= static_cast<int>(n_xyz_pts)) {
          continue;
        }
        // Check Livox tag
        if (tag_offset >= 0) {
          size_t offset = static_cast<size_t>(idx) * step + tag_offset;
          uint8_t tag_val = data_ptr[offset];
          if (tag_val != 0) {
            // skip if tag != 0
            // This is how livox mid360 tags are set (0 for valid points)
            continue;
          }
        }

        // Check angle, currenty this is hardcoded instead of robot self-filter URDF based.
        const auto& pt = xyz_cloud->points[idx];
        float angle_deg = calculateAngleAroundZ(pt.x, pt.y);

        // Filter points based on angle ranges
        if (!isAngleInRange(angle_deg, angle_ranges)) {
          next_indices.push_back(idx);
        }
      }

      // Create a new cloud with only the selected points (we pass the indices)
      pcl::PointCloud<pcl::PointXYZ>::Ptr subset_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      subset_cloud->reserve(next_indices.size());
      std::vector<int> subset_map;
      subset_map.reserve(next_indices.size());

      for (int idx : next_indices) {
        // This cloud is operating on the original indices
        subset_cloud->push_back(xyz_cloud->points[idx]);
        subset_map.push_back(idx);
      }

      // radius bsed noise
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
      ror.setInputCloud(subset_cloud);
      ror.setRadiusSearch(radius_search);
      ror.setMinNeighborsInRadius(min_neighbors);

      // Instead of returning a new cloud, we get a list of inlier indices
      std::vector<int> ror_indices;
      ror.filter(ror_indices);

      // This is the last stage of filtering.
      std::vector<int> final_indices;
      final_indices.reserve(ror_indices.size());

      for (int i : ror_indices) {
        final_indices.push_back(subset_map[i]);
      }

      // Get the points in the original format
      sensor_msgs::PointCloud2 filtered_msg;
      copyPointsByIndex(filtered_msg, *cloud_msg, final_indices);

      // Use header.stamp
      if (filtered_msg.header.stamp.isZero()) {
        ROS_ERROR("No valid header timestamp in filtered message, using bag time.");
        return -1;
      }

      // Write to output bag
      output_bag.write(output_topic, filtered_msg.header.stamp, filtered_msg);

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
    if (processed % 500 == 0) {
      ROS_INFO("Processed %lu / %lu messages", (unsigned long)processed, (unsigned long)total_msgs);
    }
  }

  input_bag.close();
  output_bag.close();
  ROS_INFO("Done writing filtered bag to %s", output_bag_path.c_str());
  return 0;
}

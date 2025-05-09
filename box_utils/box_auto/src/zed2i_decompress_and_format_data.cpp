#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <filesystem>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <boost/foreach.hpp>

#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include <compressed_depth_image_transport/rvl_codec.h>

#include <opencv2/core/core.hpp>
#define foreach BOOST_FOREACH

/*
The below license is for the following functions:

EncodeVLE
DecodeVLE
CompressRVL
DecompressRVL
decodeCompressedDepthImage

///////////////////////////////

Copyright (c) 2012, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of Willow Garage, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

int* buffer_;
int* pBuffer_;
int word_;
int nibblesWritten_;

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

static inline void EncodeVLE(int value) {
  do {
    int nibble = value & 0x7;        // lower 3 bits
    if (value >>= 3) nibble |= 0x8;  // more to come
    word_ <<= 4;
    word_ |= nibble;
    if (++nibblesWritten_ == 8)  // output word
    {
      *pBuffer_++ = word_;
      nibblesWritten_ = 0;
      word_ = 0;
    }
  } while (value);
}

static inline int DecodeVLE() {
  unsigned int nibble;
  int value = 0, bits = 29;
  do {
    if (!nibblesWritten_) {
      word_ = *pBuffer_++;  // load word
      nibblesWritten_ = 8;
    }
    nibble = word_ & 0xf0000000;
    value |= (nibble << 1) >> bits;
    word_ <<= 4;
    nibblesWritten_--;
    bits -= 3;
  } while (nibble & 0x80000000);
  return value;
}

static inline int CompressRVL(const unsigned short* input, unsigned char* output, int numPixels) {
  buffer_ = pBuffer_ = (int*)output;
  nibblesWritten_ = 0;
  const unsigned short* end = input + numPixels;
  unsigned short previous = 0;
  while (input != end) {
    int zeros = 0, nonzeros = 0;
    for (; (input != end) && !*input; input++, zeros++)
      ;
    EncodeVLE(zeros);  // number of zeros
    for (const unsigned short* p = input; (p != end) && *p++; nonzeros++)
      ;
    EncodeVLE(nonzeros);  // number of nonzeros
    for (int i = 0; i < nonzeros; i++) {
      unsigned short current = *input++;
      int delta = current - previous;
      int positive = (delta << 1) ^ (delta >> 31);
      EncodeVLE(positive);  // nonzero value
      previous = current;
    }
  }
  if (nibblesWritten_)  // last few values
    *pBuffer_++ = word_ << 4 * (8 - nibblesWritten_);
  return int((unsigned char*)pBuffer_ - (unsigned char*)buffer_);  // num bytes
}

static inline void DecompressRVL(const unsigned char* input, unsigned short* output, int numPixels) {
  buffer_ = pBuffer_ = const_cast<int*>(reinterpret_cast<const int*>(input));
  nibblesWritten_ = 0;
  unsigned short current, previous = 0;
  int numPixelsToDecode = numPixels;
  while (numPixelsToDecode) {
    int zeros = DecodeVLE();  // number of zeros
    numPixelsToDecode -= zeros;
    for (; zeros; zeros--) *output++ = 0;
    int nonzeros = DecodeVLE();  // number of nonzeros
    numPixelsToDecode -= nonzeros;
    for (; nonzeros; nonzeros--) {
      int positive = DecodeVLE();  // nonzero value
      int delta = (positive >> 1) ^ -(positive & 1);
      current = previous + delta;
      *output++ = current;
      previous = current;
    }
  }
}

static inline sensor_msgs::Image::Ptr decodeCompressedDepthImage(const sensor_msgs::CompressedImage& message) {
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message.header;

  // Assign image encoding
  const size_t split_pos = message.format.find(';');
  const std::string image_encoding = message.format.substr(0, split_pos);
  std::string compression_format;
  // Older version of compressed_depth_image_transport supports only png.
  if (split_pos == std::string::npos) {
    compression_format = "png";
  } else {
    std::string format = message.format.substr(split_pos);
    if (format.find("compressedDepth png") != std::string::npos) {
      compression_format = "png";
    } else if (format.find("compressedDepth rvl") != std::string::npos) {
      compression_format = "rvl";
    } else if (format.find("compressedDepth") != std::string::npos && format.find("compressedDepth ") == std::string::npos) {
      compression_format = "png";
    } else {
      ROS_ERROR("Unsupported image format: %s", message.format.c_str());
      return sensor_msgs::Image::Ptr();
    }
  }

  cv_ptr->encoding = image_encoding;

  // Decode message data
  if (message.data.size() > sizeof(compressed_depth_image_transport::ConfigHeader)) {
    // Read compression type from stream
    compressed_depth_image_transport::ConfigHeader compressionConfig;

    memcpy(&compressionConfig, &message.data[0], sizeof(compressionConfig));

    // Get compressed image data
    const std::vector<uint8_t> imageData(message.data.begin() + sizeof(compressionConfig), message.data.end());

    float depthZ0 = 100;
    float depthMax = 15.0;

    // Inverse depth quantization parameters
    float depthQuantA = depthZ0 * (depthZ0 + 1.0f);
    float depthQuantB = 1.0f - depthQuantA / depthMax;

    if (enc::bitDepth(image_encoding) == 32) {
      cv::Mat decompressed;
      if (compression_format == "png") {
        try {
          // Decode image data
          decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
        } catch (cv::Exception& e) {
          ROS_ERROR("%s", e.what());
          return sensor_msgs::Image::Ptr();
        }
      } else if (compression_format == "rvl") {
        const unsigned char* buffer = imageData.data();

        uint32_t cols, rows;
        memcpy(&cols, &buffer[0], 4);
        memcpy(&rows, &buffer[4], 4);
        if (rows == 0 || cols == 0) {
          ROS_ERROR_THROTTLE(1.0, "Received malformed RVL-encoded image. Size %ix%i contains zero.", cols, rows);
          return sensor_msgs::Image::Ptr();
        }

        // Sanity check - the best compression ratio is 4x; we leave some buffer, so we check whether the output image would
        // not be more than 10x larger than the compressed one. If it is, we probably received corrupted data.
        // The condition should be "numPixels * 2 > compressed.size() * 10" (because each pixel is 2 bytes), but to prevent
        // overflow, we have canceled out the *2 from both sides of the inequality.
        const auto numPixels = static_cast<uint64_t>(rows) * cols;
        if (numPixels > std::numeric_limits<int>::max() || numPixels > static_cast<uint64_t>(imageData.size()) * 5) {
          ROS_ERROR_THROTTLE(1.0, "Received malformed RVL-encoded image. It reports size %ux%u.", cols, rows);
          return sensor_msgs::Image::Ptr();
        }

        decompressed = Mat(rows, cols, CV_16UC1);
        DecompressRVL(&buffer[8], decompressed.ptr<unsigned short>(), cols * rows);
      } else {
        return sensor_msgs::Image::Ptr();
      }

      size_t rows = decompressed.rows;
      size_t cols = decompressed.cols;

      if ((rows > 0) && (cols > 0)) {
        cv_ptr->image = cv::Mat(rows, cols, CV_32FC1);

        // Depth conversion
        MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(), itDepthImg_end = cv_ptr->image.end<float>();
        MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                                          itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg) {
          // check for NaN & max depth
          if (*itInvDepthImg) {
            *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
          } else {
            *itDepthImg = std::numeric_limits<float>::quiet_NaN();
          }
        }

        // Publish message to user callback
        return cv_ptr->toImageMsg();
      }
    } else {
      // Decode raw image
      if (compression_format == "png") {
        try {
          cv_ptr->image = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
        } catch (cv::Exception& e) {
          ROS_ERROR("%s", e.what());
          return sensor_msgs::Image::Ptr();
        }
      } else if (compression_format == "rvl") {
        const unsigned char* buffer = imageData.data();
        uint32_t cols, rows;
        memcpy(&cols, &buffer[0], 4);
        memcpy(&rows, &buffer[4], 4);
        cv_ptr->image = cv::Mat(rows, cols, CV_16UC1);
        // RvlCodec rvl;
        DecompressRVL(&buffer[8], cv_ptr->image.ptr<unsigned short>(), cols * rows);
      } else {
        return sensor_msgs::Image::Ptr();
      }

      size_t rows = cv_ptr->image.rows;
      size_t cols = cv_ptr->image.cols;

      if ((rows > 0) && (cols > 0)) {
        // Publish message to user callback
        return cv_ptr->toImageMsg();
      }
    }
  }
  return sensor_msgs::Image::Ptr();
}

/*FUNCTIONS ABOVE ARE COPIED AND ALTERED FROM THE COMPRESSED DEPTH IMAGE TRANSPORT PLUGIN OF IMAGE TRANSPORT PACKGE*/

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_and_depth_save_node");
  ros::NodeHandle nh("~");

  // Get parameters
  std::string global_rosbag_path = "";
  if (!nh.getParam("global_rosbag_path", global_rosbag_path)) {
    ROS_ERROR("Parameter 'global_rosbag_path' not set!");
    return -1;
  }

  std::string depth_topic = "/gt_box/zed2i/zed_node/depth/depth_registered/compressedDepth";
  if (!nh.getParam("depth_topic", depth_topic)) {
    ROS_ERROR("Parameter 'depth_topic' not set!");
    return -1;
  }

  std::string confidence_topic = "/gt_box/zed2i/zed_node/depth/depth_registered/compressedDepth";
  if (!nh.getParam("confidence_topic", confidence_topic)) {
    ROS_ERROR("Parameter 'confidence_topic' not set!");
    return -1;
  }

  bool save_confidence = false;
  if (!nh.getParam("save_confidence", save_confidence)) {
    ROS_ERROR("Parameter 'save_confidence' not set!");
    return -1;
  }

  std::string color_topic = "/gt_box/zed2i/zed_node/left/image_rect_color/compressed";
  if (!nh.getParam("color_topic", color_topic)) {
    ROS_ERROR("Parameter 'color_topic' not set!");
    return -1;
  }

  std::string camera_info_topic = "/gt_box/zed2i/zed_node/left/camera_info";
  if (!nh.getParam("camera_info_topic", camera_info_topic)) {
    ROS_ERROR("Parameter 'camera_info_topic' not set!");
    return -1;
  }

  std::string bag_file_directory = "";
  if (!nh.getParam("bag_file_directory", bag_file_directory)) {
    ROS_ERROR("Parameter 'bag_file_directory' not set!");
    return -1;
  }

  int max_color_images = 50;
  if (!nh.getParam("max_color_images", max_color_images)) {
    ROS_ERROR("Parameter 'max_color_images' not set!");
    return -1;
  }
  const int max_depth_images = max_color_images;

  int frames_to_skip = 50;
  if (!nh.getParam("frames_to_skip", frames_to_skip)) {
    ROS_ERROR("Parameter 'frames_to_skip' not set!");
    return -1;
  }

  bool saveAsEXR = false;
  if (!nh.getParam("save_as_exr", saveAsEXR)) {
    ROS_ERROR("Parameter 'save_as_exr' not set!");
    return -1;
  }

  if (saveAsEXR) {
    ROS_ERROR("################################");
    ROS_ERROR("#### EXR format is enabled #####");
    ROS_ERROR("################################");
  }

  std::string folder_path = "GrandTourBestTour";
  if (!nh.getParam("output_folder_path", folder_path)) {
    ROS_ERROR("Parameter 'output_folder_path' not set!");
    return -1;
  }

  // Check if the folder exists; if not, create it
  if (!std::filesystem::exists(folder_path)) {
    if (!std::filesystem::create_directories(folder_path)) {
      ROS_ERROR("Failed to create directory: %s", folder_path.c_str());
      return -1;
    }
  }

  // Clear the files and any subfolders in the folder using std::filesystem
  for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
    std::filesystem::remove_all(entry.path());
  }

  // Create "images" subfolder if it doesn't exist
  std::filesystem::path images_folder = std::filesystem::path(folder_path) / "images";
  if (!std::filesystem::exists(images_folder)) {
    if (!std::filesystem::create_directories(images_folder)) {
      ROS_ERROR("Failed to create directory: %s", images_folder.string().c_str());
      return -1;
    }
  }

  // Create "depth_images" subfolder if it doesn't exist
  std::filesystem::path depth_images_folder = std::filesystem::path(folder_path) / "depth_images";
  if (!std::filesystem::exists(depth_images_folder)) {
    if (!std::filesystem::create_directories(depth_images_folder)) {
      ROS_ERROR("Failed to create directory: %s", depth_images_folder.string().c_str());
      return -1;
    }
  }

  // Create "confidence_images" subfolder if it doesn't exist
  std::filesystem::path confidence_folder = std::filesystem::path(folder_path) / "confidence_images";

  if (save_confidence) {
    if (!std::filesystem::exists(confidence_folder)) {
      if (!std::filesystem::create_directories(confidence_folder)) {
        ROS_ERROR("Failed to create directory: %s", confidence_folder.string().c_str());
        return -1;
      }
    }
  }

  ROS_INFO("Images folder: %s", images_folder.string().c_str());
  ROS_INFO("Depth images folder: %s", depth_images_folder.string().c_str());

  std::string map_frame = "dlio_map";  // map_o3d world
  if (!nh.getParam("map_frame", map_frame)) {
    ROS_ERROR("Parameter 'map_frame' not set!");
    return -1;
  }

  std::string rgbBagPath = "";
  std::string depthBagPath = "";

  if (!global_rosbag_path.empty()) {
    rgbBagPath = global_rosbag_path;
    depthBagPath = global_rosbag_path;
  } else {
    {
      // Substring to look for
      std::string target_substring = "_depth.bag";

      bool found_any = false;

      try {
        // Iterate over the directory
        for (const auto& entry : std::filesystem::directory_iterator(bag_file_directory)) {
          if (!entry.is_regular_file()) {
            continue;  // Skip directories/symlinks/etc.
          }

          const std::string filename = entry.path().filename().string();

          // Check if file name contains the desired substring
          if (filename.find(target_substring) != std::string::npos) {
            if (found_any) {
              // We already found a match before -> multiple files exist
              std::cerr << "Error: Found multiple files containing '" << target_substring << "' in directory.\n";
              return 1;  // or throw an exception
            }
            // Record this file as our match
            depthBagPath = entry.path().string();
            found_any = true;
          }
        }

        // After iterating, check our results
        if (!found_any) {
          std::cerr << "Error: No files containing '" << target_substring << "' found in directory.\n";
          return -1;  // or throw an exception
        }

        // Exactly one match was found
        std::cout << "Matched file path: " << depthBagPath << std::endl;

      } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
        return -1;  // or throw
      }
    }

    {
      // Substring to look for
      std::string target_substring = "_images.bag";

      bool found_any = false;

      try {
        // Iterate over the directory
        for (const auto& entry : std::filesystem::directory_iterator(bag_file_directory)) {
          if (!entry.is_regular_file()) {
            continue;  // Skip directories/symlinks/etc.
          }

          const std::string filename = entry.path().filename().string();

          // Check if file name contains the desired substring
          if (filename.find(target_substring) != std::string::npos) {
            if (found_any) {
              // We already found a match before -> multiple files exist
              std::cerr << "Error: Found multiple files containing '" << target_substring << "' in directory.\n";
              return 1;
            }

            rgbBagPath = entry.path().string();
            found_any = true;
          }
        }

        // After iterating, check our results
        if (!found_any) {
          std::cerr << "Error: No files containing '" << target_substring << "' found in directory.\n";
          return -1;  // or throw an exception
        }

        // Exactly one match was found
        std::cout << "Matched file path: " << rgbBagPath << std::endl;

      } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
        return -1;
      }
    }
  }

  rosbag::Bag depthBag;
  rosbag::Bag rgbBag;
  try {
    ROS_INFO("Opening bag file: %s", rgbBagPath.c_str());
    rgbBag.open(rgbBagPath, rosbag::bagmode::Read);

    ROS_INFO("Opening bag file: %s", depthBagPath.c_str());
    depthBag.open(depthBagPath, rosbag::bagmode::Read);

  } catch (rosbag::BagException& e) {
    ROS_ERROR("Failed to open bag file: %s", e.what());
    return -1;
  }

  std::vector<std::string> tfContainingBags;
  nh.getParam("/tf_containing_bag_paths", tfContainingBags);

  if (tfContainingBags.empty()) {
    ROS_ERROR_STREAM("No TF Input bag is provided.");
    ros::shutdown();
    return -1;
  }

  ROS_WARN("Reading full rgbBag file to get the duration of the tf buffer");
  rosbag::View full_view(rgbBag);

  ros::Time start_time = full_view.getBeginTime();
  ros::Time end_time = full_view.getEndTime();
  ros::Duration bag_duration = end_time - start_time;

  // Initialize TF Buffer with the duration of the bag
  tf2_ros::Buffer tf_buffer(bag_duration);

  /////////////////////////
  ros::Time lastDepthImageTime;

  ROS_WARN("Getting depth");
  rosbag::View depth_view(depthBag, rosbag::TopicQuery(depth_topic));

  int saved_depth_count = 0;
  int nbDepthFrames = 0;
  bool found_depth = false;

  foreach (rosbag::MessageInstance const m, depth_view) {
    if (saved_depth_count >= max_depth_images) {
      break;
    }

    sensor_msgs::CompressedImage::ConstPtr cimg = m.instantiate<sensor_msgs::CompressedImage>();
    if (cimg) {
      ++nbDepthFrames;
      if (nbDepthFrames < frames_to_skip) {
        continue;
      }

      lastDepthImageTime = cimg->header.stamp;
      found_depth = true;

      // Decode compressed depth image
      sensor_msgs::Image::Ptr decompressed = decodeCompressedDepthImage(*cimg);

      try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv::Mat mat;
        try {
          // Convert to CV_32FC1 as we assume myDecode returns a float image
          cv_ptr = cv_bridge::toCvCopy(*decompressed, "32FC1");
          mat = cv_ptr->image.clone();  // 32-bit float depth in meters (assumed)
        } catch (cv_bridge::Exception& e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return -1;
        }

        if (saveAsEXR) {
          std::ostringstream filename;
          filename << "depth" << saved_depth_count << ".exr";
          if (cv::imwrite(folder_path + "/" + filename.str(), mat)) {
            ROS_INFO("Saved %s", filename.str().c_str());
            saved_depth_count++;
          } else {
            ROS_ERROR("Failed to write %s", filename.str().c_str());
            return -1;
          }

        } else {
          // Convert 32F (meters) to 16U (millimeters) for PNG saving
          cv::Mat depth_16u;
          mat.convertTo(depth_16u, CV_16UC1, 1000.0);  // Scale meters to millimeters
          std::ostringstream filename;
          filename << "depth" << saved_depth_count << ".png";
          if (cv::imwrite(depth_images_folder.string() + "/" + filename.str(), depth_16u)) {
            ROS_INFO("Saved %s", filename.str().c_str());
            saved_depth_count++;
          } else {
            ROS_ERROR("Failed to write %s", filename.str().c_str());
          }
        }

      } catch (std::exception& e) {
        ROS_ERROR("Error processing depth image: %s", e.what());
      }
    }

    if (!found_depth) {
      ROS_WARN("No compressed depth images found on topic '%s' in bag '%s'", depth_topic.c_str(), depthBagPath.c_str());
      return -1;
    } else if (saved_depth_count == 0) {
      ROS_WARN("Found compressed depth images but failed to decode or save any.");
      return -1;
    }
  }

  if (save_confidence) {
    /////////////////////////
    ros::Time lastConfidenceImageTime;

    ROS_WARN("Getting Confidence");
    rosbag::View confidence_view(depthBag, rosbag::TopicQuery(confidence_topic));

    int saved_confidence_count = 0;
    int nbConfidenceFrames = 0;
    bool found_confidence = false;

    foreach (rosbag::MessageInstance const m, confidence_view) {
      if (saved_confidence_count >= max_depth_images) {
        break;
      }

      sensor_msgs::CompressedImage::ConstPtr cimg = m.instantiate<sensor_msgs::CompressedImage>();
      if (cimg) {
        ++nbConfidenceFrames;
        if (nbConfidenceFrames < frames_to_skip) {
          continue;
        }

        lastConfidenceImageTime = cimg->header.stamp;
        found_confidence = true;

        // Decode compressed depth image
        sensor_msgs::Image::Ptr decompressed = decodeCompressedDepthImage(*cimg);

        try {
          cv_bridge::CvImageConstPtr cv_ptr;
          cv::Mat mat;
          try {
            // Convert to CV_32FC1 as we assume myDecode returns a float image
            cv_ptr = cv_bridge::toCvCopy(*decompressed, "32FC1");
            mat = cv_ptr->image.clone();  // 32-bit float depth in meters (assumed)
          } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return -1;
          }

          if (saveAsEXR) {
            std::ostringstream filename;
            filename << "depth" << saved_confidence_count << ".exr";
            if (cv::imwrite(folder_path + "/" + filename.str(), mat)) {
              ROS_INFO("Saved %s", filename.str().c_str());
              saved_confidence_count++;
            } else {
              ROS_ERROR("Failed to write %s", filename.str().c_str());
              return -1;
            }

          } else {
            // Convert 32F (meters) to 16U (millimeters) for PNG saving
            cv::Mat depth_16u;
            mat.convertTo(depth_16u, CV_16UC1, 1000.0);  // Scale meters to millimeters
            std::ostringstream filename;
            filename << "confidence" << saved_confidence_count << ".png";
            if (cv::imwrite(confidence_folder.string() + "/" + filename.str(), depth_16u)) {
              ROS_INFO("Saved %s", filename.str().c_str());
              saved_confidence_count++;
            } else {
              ROS_ERROR("Failed to write %s", filename.str().c_str());
            }
          }

        } catch (std::exception& e) {
          ROS_ERROR("Error processing depth image: %s", e.what());
        }
      }

      if (!found_confidence) {
        ROS_WARN("No compressed confidence images found on topic '%s' in bag '%s'", confidence_topic.c_str(), depthBagPath.c_str());
        return -1;
      } else if (saved_confidence_count == 0) {
        ROS_WARN("Found compressed confidence images but failed to decode or save any.");
        return -1;
      }
    }
  }

  // Create a vector to hold the opened bags
  std::vector<rosbag::Bag> bags;
  bags.reserve(tfContainingBags.size());

  ROS_INFO_STREAM("Number of Bags for TF: " << tfContainingBags.size());

  // Open all bags
  for (const auto& path : tfContainingBags) {
    ROS_INFO_STREAM("Opening bag: " << path);
    // rosbag::Bag tfbag;
    // bags.emplace_back(std::move(tfbag));

    bags.emplace_back();

    // tfbag.open(path, rosbag::bagmode::Read);
    try {
      bags.back().open(path, rosbag::bagmode::Read);
    } catch (const std::exception& e) {
      bags.back().open(bag_file_directory + path, rosbag::bagmode::Read);
    }
  }

  // Topics we care about
  std::vector<std::string> tf_topics{"/tf", "/tf_static"};

  // Create a combined view for all /tf and /tf_static from all bags
  rosbag::View combined_view;
  for (auto& tfbag : bags) {
    combined_view.addQuery(tfbag, rosbag::TopicQuery(tf_topics));
  }

  double tf_timeout_seconds = 1.0;
  bool got_tf_static = false;

  // Iterate over the combined view
  for (const rosbag::MessageInstance& m : combined_view) {
    const std::string& topic_name = m.getTopic();

    // If it's a /tf message
    if (topic_name == "/tf" || topic_name == "tf") {
      tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
      if (tf_msg) {
        // Break if beyond our time limit
        if (m.getTime() > lastDepthImageTime + ros::Duration(tf_timeout_seconds)) {
          ROS_WARN_STREAM("Exceeded TF time limit. Breaking...");
          break;
        }
        // Insert transforms
        for (const geometry_msgs::TransformStamped& transform : tf_msg->transforms) {
          tf_buffer.setTransform(transform, "default_authority", false);
        }
      }

      // If it's a /tf_static message
    } else if (topic_name == "/tf_static") {
      // Often, we only need /tf_static once
      if (!got_tf_static) {
        tf2_msgs::TFMessage::ConstPtr tf_static_msg = m.instantiate<tf2_msgs::TFMessage>();
        if (tf_static_msg) {
          for (const geometry_msgs::TransformStamped& transform : tf_static_msg->transforms) {
            // static = true
            tf_buffer.setTransform(transform, "default_authority", true);
          }
          ROS_INFO_STREAM("Got tf_static transforms.");
          got_tf_static = true;
        } else {
          ROS_ERROR_STREAM("Failed to get tf_static message.");
          throw std::runtime_error("Failed to instantiate tf_static");
        }
        // If you want to stop reading /tf_static after the first time:
        // break;
      }
    }
  }

  // Close all bags
  for (auto& bag : bags) {
    if (bag.isOpen()) {
      bag.close();
    }
  }

  ROS_INFO_STREAM("Done processing TF data from all bags.");

  ROS_WARN("Getting color");

  rosbag::View color_view(rgbBag, rosbag::TopicQuery(color_topic));
  int saved_color_count = 0;
  int nbRGBFrames = 0;
  bool found_color = false;

  // Second pass: read the odometry messages and match timestamps
  std::string output_file = folder_path + "/traj.txt";
  std::ofstream ofs(output_file);
  if (!ofs.is_open()) {
    ROS_ERROR("Failed to open output file: %s", output_file.c_str());
    return -1;
  }

  foreach (rosbag::MessageInstance const m, color_view) {
    if (saved_color_count >= max_color_images) {
      break;
    }

    sensor_msgs::CompressedImage::ConstPtr cimg = m.instantiate<sensor_msgs::CompressedImage>();
    if (cimg) {
      found_color = true;

      ++nbRGBFrames;
      if (nbRGBFrames < frames_to_skip) {
        continue;
      }

      std::string format = cimg->format;

      // TODO, if original compression is PNG, save image as PNG.
      // bool is_png = (format.find("png") != std::string::npos);
      bool is_rgb8 = (format.find("rgb8") != std::string::npos);

      // Decode PNG from compressed data
      cv::Mat rawData(1, (int)cimg->data.size(), CV_8UC1, (void*)cimg->data.data());
      cv::Mat color_img = cv::imdecode(rawData, cv::IMREAD_COLOR);
      if (color_img.empty()) {
        ROS_ERROR("Failed to decode color image from topic '%s'", color_topic.c_str());
        return -1;
      }

      // If the original encoding is rgb8, we must convert from BGR (OpenCV default) to RGB
      if (is_rgb8) {
        cv::Mat color_img_bgr;
        cv::cvtColor(color_img, color_img_bgr, cv::COLOR_RGB2BGR);
        color_img = color_img_bgr;
      }

      std::ostringstream filename;
      filename << "frame" << saved_color_count << ".jpg";
      if (cv::imwrite(images_folder.string() + "/" + filename.str(), color_img)) {
        ROS_INFO("Saved %s", filename.str().c_str());
        saved_color_count++;
      } else {
        ROS_ERROR("Failed to write %s", filename.str().c_str());
      }

      try {
        {
          geometry_msgs::TransformStamped lookup_transform;
          // lookup_transform =
          //         tf_buffer.lookupTransform("dlio_map", cimg->header.stamp, "zed2i_left_camera_optical_frame", cimg->header.stamp,
          //         "dlio_map", ros::Duration(0.1));
          ros::spinOnce();
          lookup_transform = tf_buffer.lookupTransform(map_frame, cimg->header.frame_id, cimg->header.stamp);
          ros::spinOnce();

          tf2::Quaternion q(lookup_transform.transform.rotation.x, lookup_transform.transform.rotation.y,
                            lookup_transform.transform.rotation.z, lookup_transform.transform.rotation.w);
          tf2::Matrix3x3 R(q);

          double r11 = R[0][0], r12 = R[0][1], r13 = R[0][2];
          double r21 = R[1][0], r22 = R[1][1], r23 = R[1][2];
          double r31 = R[2][0], r32 = R[2][1], r33 = R[2][2];

          double px = lookup_transform.transform.translation.x;
          double py = lookup_transform.transform.translation.y;
          double pz = lookup_transform.transform.translation.z;

          double T_map_camera[16];

          T_map_camera[0] = r11;
          T_map_camera[1] = r12;
          T_map_camera[2] = r13;
          T_map_camera[3] = px;
          T_map_camera[4] = r21;
          T_map_camera[5] = r22;
          T_map_camera[6] = r23;
          T_map_camera[7] = py;
          T_map_camera[8] = r31;
          T_map_camera[9] = r32;
          T_map_camera[10] = r33;
          T_map_camera[11] = pz;
          T_map_camera[12] = 0;
          T_map_camera[13] = 0;
          T_map_camera[14] = 0;
          T_map_camera[15] = 1;

          // Construct 4x4 matrix in row-major order
          // format: r11 r12 r13 px r21 r22 r23 py r31 r32 r33 pz 0 0 0 1
          for (int i = 0; i < 16; i++) {
            ofs << T_map_camera[i];
            if (i < 15) ofs << " ";
          }
          ofs << "\n";
        }

      } catch (std::exception& e) {
        ROS_ERROR("Error processing color image: %s", e.what());
        return -1;
      }
    }
  }

  if (!found_color) {
    ROS_WARN("No compressed color images found on topic '%s' in bag '%s'", color_topic.c_str(), rgbBagPath.c_str());
    return -1;
  } else if (saved_color_count == 0) {
    ROS_WARN("Found compressed color images but failed to decode or save any.");
    return -1;
  }
  ofs.close();

  // Write camera info
  rosbag::View view(rgbBag, rosbag::TopicQuery(camera_info_topic));

  bool found_camera_info = false;

  foreach (rosbag::MessageInstance const m, view) {
    sensor_msgs::CameraInfo::ConstPtr cam_info = m.instantiate<sensor_msgs::CameraInfo>();
    if (cam_info) {
      found_camera_info = true;

      // Extract intrinsic parameters
      int w = cam_info->width;
      int h = cam_info->height;
      double fx = cam_info->K[0];
      double fy = cam_info->K[4];
      double cx = cam_info->K[2];
      double cy = cam_info->K[5];

      // Compute the scale based on saveAsEXR
      int scale = saveAsEXR ? 1 : 1000;

      // Write the JSON to a file
      std::ofstream ofs(folder_path + "/camera_info.json");
      if (!ofs.is_open()) {
        ROS_ERROR("Failed to open output file for camera info.");
      } else {
        ofs << "{\n";
        ofs << "    \"camera\": {\n";
        ofs << "        \"w\": " << w << ",\n";
        ofs << "        \"h\": " << h << ",\n";
        ofs << "        \"fx\": " << fx << ",\n";
        ofs << "        \"fy\": " << fy << ",\n";
        ofs << "        \"cx\": " << cx << ",\n";
        ofs << "        \"cy\": " << cy << ",\n";
        ofs << "        \"scale\": " << scale << "\n";
        ofs << "    }\n";
        ofs << "}\n";
        ofs.close();
        ROS_INFO("Saved camera info to camera_info.json");
      }

      // Since we only want to save it once, break out
      break;
    }
  }

  if (!found_camera_info) {
    ROS_WARN("No camera info found on topic '%s' in bag '%s'", camera_info_topic.c_str(), rgbBagPath.c_str());
  }

  rgbBag.close();
  depthBag.close();

  ROS_WARN("Completed successfully. Self-terminating.");
  ros::shutdown();
  return 0;
}

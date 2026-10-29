#include <box_export/atomic_directory.hpp>
// zed2i_decompression_and_saving.cpp
//
// Exports (RGB):
//   Stereo (right+left):
//     <output_folder_path>/images/right/frameX.jpg
//     <output_folder_path>/images/left/frameX.jpg
//   Mono (single RGB stream):
//     <output_folder_path>/image/frameX.jpg
//
// Exports (Depth / Confidence) [OPTIONAL]:
//   <output_folder_path>/depth_images/depthX.(png|exr)            [if depth enabled+available]
//   <output_folder_path>/confidence_images/confidenceX.(png|exr)  [if save_confidence==true and available]
//
// Trajectories:
//   Stereo:
//     <output_folder_path>/traj_right.txt   pose of RIGHT camera frame in map_frame
//     <output_folder_path>/traj_left.txt    pose of LEFT  camera frame in map_frame
//   Mono:
//     <output_folder_path>/traj.txt         pose of the (single) RGB camera frame in map_frame
//
// Depth/confidence are read as sensor_msgs::Image OR sensor_msgs::CompressedImage.
// CompressedImage supports BOTH:
//   - standard compressed_image_transport payloads (png/jpeg/...) via cv::imdecode
//   - compressedDepth transport payloads (png or rvl) by decoding the compressedDepth header + payload
//
// Stereo pairing: RIGHT+LEFT are paired by exact header.stamp equality.
//
// TF accumulation fix: loads /tf_static always, filters /tf by TransformStamped.header.stamp
// within the RGB bag time window.
//
// Build fix: rosbag::View is non-assignable (operator= is private). Confidence view selection
// is done by picking the topic first, then constructing the rosbag::View once.
//
// CameraInfo JSON export (UPDATED):
//   Writes full CameraInfo, including distortion_model, D, K, R, P, binning, ROI, header stamp/frame_id.
//   If left_camera_info_topic is provided and has messages, also writes LEFT into the same JSON.

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <string>
#include <vector>
#include <limits>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>

#include <std_msgs/Header.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <tf2_msgs/TFMessage.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace fs = std::filesystem;

// -----------------------------
// Colorful logging + progress
// -----------------------------
namespace ansi {
static constexpr const char* kReset   = "\033[0m";
static constexpr const char* kBold    = "\033[1m";
static constexpr const char* kDim     = "\033[2m";

static constexpr const char* kRed     = "\033[31m";
static constexpr const char* kGreen   = "\033[32m";
static constexpr const char* kYellow  = "\033[33m";
static constexpr const char* kBlue    = "\033[34m";
static constexpr const char* kMagenta = "\033[35m";
static constexpr const char* kCyan    = "\033[36m";
static constexpr const char* kWhite   = "\033[37m";
}  // namespace ansi

static inline std::string matTypeToString(int type) {
  const int depth = type & CV_MAT_DEPTH_MASK;
  const int chans = 1 + (type >> CV_CN_SHIFT);
  std::string r;

  switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }
  r += "C" + std::to_string(chans);
  return r;
}

static inline void printProgressBar(const std::string& label, size_t done, size_t total) {
  const int width = 40;
  const double ratio = (total == 0) ? 1.0 : static_cast<double>(done) / static_cast<double>(total);
  const int filled = static_cast<int>(std::round(ratio * width));
  const int pct = static_cast<int>(std::round(ratio * 100.0));

  std::cout << "\r"
            << ansi::kDim << label << ansi::kReset << " "
            << ansi::kCyan << "[" << std::string(filled, '#') << std::string(width - filled, ' ') << "]"
            << ansi::kReset << " "
            << ansi::kBold << std::setw(3) << pct << "%" << ansi::kReset
            << " (" << done << "/" << total << ")"
            << std::flush;

  if (done >= total) std::cout << "\n";
}

static inline std::string timeToString(const ros::Time& t) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(9) << t.toSec();
  return ss.str();
}

static inline void logStreamHeaderOnce(const std::string& stream_name,
                                       const std::string& topic,
                                       const std::string& msg_type,
                                       const std::string& encoding_or_format,
                                       const std::string& frame_id) {
  std::cout << ansi::kBold << ansi::kMagenta << "[STREAM]" << ansi::kReset << " "
            << ansi::kBold << stream_name << ansi::kReset
            << " topic=" << ansi::kCyan << topic << ansi::kReset
            << " msg=" << ansi::kYellow << msg_type << ansi::kReset
            << " enc/format=" << ansi::kGreen << encoding_or_format << ansi::kReset
            << " frame_id=" << ansi::kBlue << (frame_id.empty() ? "<empty>" : frame_id) << ansi::kReset
            << "\n";
}

static inline std::string toLowerCopy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

static inline std::string trimCopy(const std::string& s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}

static inline std::string guessRgbBgr(const std::string& s_in) {
  const std::string s = toLowerCopy(s_in);
  if (s.find("rgb8") != std::string::npos) return "rgb8";
  if (s.find("bgr8") != std::string::npos) return "bgr8";
  return "unknown";
}

static inline bool shouldLogFailure(size_t& counter, size_t every_n = 100) {
  ++counter;
  return (counter <= 5) || (every_n > 0 && (counter % every_n == 0));
}

// -----------------------------
// Bag matching helper (substring)
// -----------------------------
static bool findSingleBagBySubstring(const std::string& directory,
                                     const std::string& substring,
                                     std::string& out_path) {
  out_path.clear();
  if (directory.empty()) {
    throw std::runtime_error("bag_file_directory is empty (cannot search).");
  }
  if (substring.empty()) {
    return false;
  }

  bool found_any = false;
  try {
    for (const auto& entry : fs::directory_iterator(directory)) {
      if (!entry.is_regular_file()) continue;
      const std::string filename = entry.path().filename().string();
      if (filename.find(substring) != std::string::npos) {
        if (found_any) {
          std::ostringstream oss;
          oss << "Found multiple files containing '" << substring << "' in directory '" << directory << "'";
          throw std::runtime_error(oss.str());
        }
        out_path = entry.path().string();
        found_any = true;
      }
    }
  } catch (const fs::filesystem_error& e) {
    std::ostringstream oss;
    oss << "Filesystem error while scanning '" << directory << "': " << e.what();
    throw std::runtime_error(oss.str());
  }

  return found_any;
}

// -----------------------------
// compressedDepth (png/rvl) decoding
// -----------------------------
class RvlCodec {
public:
  void DecompressRVL(const unsigned char* input, unsigned short* output, int numPixels) {
    pBuffer_ = const_cast<int*>(reinterpret_cast<const int*>(input));
    nibblesWritten_ = 0;
    unsigned short current = 0, previous = 0;

    int numPixelsToDecode = numPixels;
    while (numPixelsToDecode > 0) {
      int zeros = DecodeVLE();
      numPixelsToDecode -= zeros;
      for (; zeros > 0; --zeros) *output++ = 0;

      int nonzeros = DecodeVLE();
      numPixelsToDecode -= nonzeros;
      for (; nonzeros > 0; --nonzeros) {
        int positive = DecodeVLE();
        int delta = (positive >> 1) ^ -(positive & 1);
        current = static_cast<unsigned short>(previous + delta);
        *output++ = current;
        previous = current;
      }
    }
  }

private:
  int DecodeVLE() {
    unsigned int nibble;
    int value = 0, bits = 29;
    do {
      if (!nibblesWritten_) {
        word_ = *pBuffer_++;  // load word
        nibblesWritten_ = 8;
      }
      nibble = static_cast<unsigned int>(word_) & 0xf0000000;
      value |= static_cast<int>((nibble << 1) >> bits);
      word_ <<= 4;
      nibblesWritten_--;
      bits -= 3;
    } while (nibble & 0x80000000);
    return value;
  }

  int* pBuffer_{nullptr};
  int word_{0};
  int nibblesWritten_{0};
};

struct CompressedDepthConfigHeader {
  int32_t format;
  float depthParam[2];  // depthQuantA, depthQuantB
};

static inline bool isCompressedDepthFormat(const std::string& fmt) {
  return toLowerCopy(fmt).find("compresseddepth") != std::string::npos;
}

static inline bool parseCompressedDepthFormat(const std::string& fmt,
                                              std::string& image_encoding_out,
                                              std::string& compression_out /*png|rvl*/) {
  const size_t split_pos = fmt.find(';');
  image_encoding_out = trimCopy(fmt.substr(0, split_pos));

  if (split_pos == std::string::npos) {
    compression_out = "png";
    return true;
  }

  const std::string tail = toLowerCopy(fmt.substr(split_pos));
  if (tail.find("compresseddepth png") != std::string::npos) {
    compression_out = "png";
    return true;
  }
  if (tail.find("compresseddepth rvl") != std::string::npos) {
    compression_out = "rvl";
    return true;
  }
  if (tail.find("compresseddepth") != std::string::npos) {
    compression_out = "png";
    return true;
  }
  return false;
}

static bool decodeCompressedDepthToMat(const sensor_msgs::CompressedImage& msg,
                                       cv::Mat& out,
                                       std::string& encoding_out,
                                       size_t& fail_counter) {
  out.release();
  encoding_out.clear();

  std::string compression_format;
  if (!parseCompressedDepthFormat(msg.format, encoding_out, compression_format)) {
    if (shouldLogFailure(fail_counter)) {
      ROS_ERROR("Unsupported compressedDepth format string: '%s'", msg.format.c_str());
    }
    return false;
  }

  if (msg.data.size() <= sizeof(CompressedDepthConfigHeader)) {
    if (shouldLogFailure(fail_counter)) {
      ROS_ERROR("compressedDepth message too small: %zu bytes (need > %zu).",
                msg.data.size(), sizeof(CompressedDepthConfigHeader));
    }
    return false;
  }

  CompressedDepthConfigHeader cfg{};
  std::memcpy(&cfg, msg.data.data(), sizeof(cfg));
  const float depthQuantA = cfg.depthParam[0];
  const float depthQuantB = cfg.depthParam[1];

  const uint8_t* imageData = msg.data.data() + sizeof(cfg);
  const size_t imageSize = msg.data.size() - sizeof(cfg);
  if (imageSize == 0) {
    if (shouldLogFailure(fail_counter)) {
      ROS_ERROR("compressedDepth has empty payload after header.");
    }
    return false;
  }

  const std::string enc_l = toLowerCopy(encoding_out);
  const bool is_32f = (enc_l.find("32f") != std::string::npos);
  const bool is_16u = (enc_l.find("16u") != std::string::npos);

  if (!is_32f && !is_16u) {
    if (shouldLogFailure(fail_counter)) {
      ROS_ERROR("compressedDepth unsupported image encoding='%s' (expected 32FC1 or 16UC1).", encoding_out.c_str());
    }
    return false;
  }

  if (compression_format == "png") {
    try {
      cv::Mat raw(1, static_cast<int>(imageSize), CV_8UC1, const_cast<uint8_t*>(imageData));
      cv::Mat decoded = cv::imdecode(raw, cv::IMREAD_UNCHANGED);
      if (decoded.empty()) {
        if (shouldLogFailure(fail_counter)) {
          ROS_ERROR("cv::imdecode failed for compressedDepth png (encoding=%s).", encoding_out.c_str());
        }
        return false;
      }

      if (is_32f) {
        if (decoded.type() != CV_16UC1) {
          if (shouldLogFailure(fail_counter)) {
            ROS_ERROR("compressedDepth png expected decoded type=16UC1 for 32FC1 encoding, got %s",
                      matTypeToString(decoded.type()).c_str());
          }
          return false;
        }

        out = cv::Mat(decoded.rows, decoded.cols, CV_32FC1);
        for (int r = 0; r < decoded.rows; ++r) {
          const uint16_t* invp = decoded.ptr<uint16_t>(r);
          float* zp = out.ptr<float>(r);
          for (int c = 0; c < decoded.cols; ++c) {
            const uint16_t v = invp[c];
            if (v) {
              zp[c] = depthQuantA / (static_cast<float>(v) - depthQuantB);
            } else {
              zp[c] = std::numeric_limits<float>::quiet_NaN();
            }
          }
        }
      } else {
        out = decoded;
      }
      return true;
    } catch (const cv::Exception& e) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("cv::imdecode exception (compressedDepth png): %s", e.what());
      }
      return false;
    }
  }

  if (compression_format == "rvl") {
    if (imageSize < 8) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("compressedDepth rvl payload too small: %zu bytes.", imageSize);
      }
      return false;
    }

    uint32_t cols = 0, rows = 0;
    std::memcpy(&cols, imageData + 0, 4);
    std::memcpy(&rows, imageData + 4, 4);
    if (cols == 0 || rows == 0) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("compressedDepth rvl malformed image size: %ux%u", cols, rows);
      }
      return false;
    }

    const uint64_t numPixels = static_cast<uint64_t>(rows) * static_cast<uint64_t>(cols);
    if (numPixels > static_cast<uint64_t>(std::numeric_limits<int>::max())) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("compressedDepth rvl too many pixels: %llu", static_cast<unsigned long long>(numPixels));
      }
      return false;
    }

    cv::Mat decoded16(static_cast<int>(rows), static_cast<int>(cols), CV_16UC1);
    RvlCodec rvl;
    rvl.DecompressRVL(imageData + 8, decoded16.ptr<uint16_t>(), static_cast<int>(numPixels));

    if (is_32f) {
      out = cv::Mat(decoded16.rows, decoded16.cols, CV_32FC1);
      for (int r = 0; r < decoded16.rows; ++r) {
        const uint16_t* invp = decoded16.ptr<uint16_t>(r);
        float* zp = out.ptr<float>(r);
        for (int c = 0; c < decoded16.cols; ++c) {
          const uint16_t v = invp[c];
          if (v) {
            zp[c] = depthQuantA / (static_cast<float>(v) - depthQuantB);
          } else {
            zp[c] = std::numeric_limits<float>::quiet_NaN();
          }
        }
      }
    } else {
      out = decoded16;
    }
    return true;
  }

  if (shouldLogFailure(fail_counter)) {
    ROS_ERROR("compressedDepth: unsupported compression_format='%s'", compression_format.c_str());
  }
  return false;
}

static bool decodeCompressedAnyToMat(const sensor_msgs::CompressedImage& msg,
                                     cv::Mat& out,
                                     std::string& encoding_or_format_out,
                                     size_t& fail_counter) {
  out.release();
  encoding_or_format_out = msg.format;

  if (isCompressedDepthFormat(msg.format)) {
    std::string enc;
    if (!decodeCompressedDepthToMat(msg, out, enc, fail_counter)) {
      return false;
    }
    encoding_or_format_out = enc + "; compressedDepth";
    return true;
  }

  if (msg.data.empty()) return false;
  try {
    cv::Mat buf(1, static_cast<int>(msg.data.size()), CV_8UC1, const_cast<uint8_t*>(msg.data.data()));
    out = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
    return !out.empty();
  } catch (const cv::Exception& e) {
    if (shouldLogFailure(fail_counter)) {
      ROS_ERROR("cv::imdecode exception (standard CompressedImage): %s (format='%s')", e.what(), msg.format.c_str());
    }
    return false;
  }
}

// -----------------------------
// Depth conversion to meters (float32)
// -----------------------------
static bool depthMatToFloatMeters(const cv::Mat& in, cv::Mat& out_meters) {
  if (in.empty()) return false;

  if (in.type() == CV_32FC1) {
    out_meters = in.clone();
    return true;
  }

  if (in.type() == CV_16UC1) {
    in.convertTo(out_meters, CV_32FC1, 1.0 / 1000.0);  // mm -> m
    return true;
  }

  ROS_ERROR("Unsupported depth Mat type %d. Expected CV_16UC1 (mm) or CV_32FC1 (m).", in.type());
  return false;
}

static bool loadDepthMetersFromMI(const rosbag::MessageInstance& mi,
                                  cv::Mat& depth_meters,
                                  std_msgs::Header& hdr,
                                  size_t& fail_counter) {
  if (auto img = mi.instantiate<sensor_msgs::Image>()) {
    hdr = img->header;

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(img, img->encoding);
    } catch (const cv_bridge::Exception& e) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("Depth cv_bridge exception: %s (encoding='%s' stamp=%s)",
                  e.what(), img->encoding.c_str(), timeToString(img->header.stamp).c_str());
      }
      return false;
    }

    if (!depthMatToFloatMeters(cv_ptr->image, depth_meters)) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("Depth conversion failed (encoding='%s' type=%s stamp=%s)",
                  img->encoding.c_str(), matTypeToString(cv_ptr->image.type()).c_str(),
                  timeToString(img->header.stamp).c_str());
      }
      return false;
    }
    return true;
  }

  if (auto cimg = mi.instantiate<sensor_msgs::CompressedImage>()) {
    hdr = cimg->header;

    cv::Mat decoded;
    std::string decoded_kind;
    if (!decodeCompressedAnyToMat(*cimg, decoded, decoded_kind, fail_counter)) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("Failed to decode depth CompressedImage (format='%s', bytes=%zu, stamp=%s).",
                  cimg->format.c_str(), cimg->data.size(), timeToString(cimg->header.stamp).c_str());
      }
      return false;
    }

    if (!depthMatToFloatMeters(decoded, depth_meters)) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("Depth conversion failed after decode (format='%s' decoded=%s stamp=%s).",
                  cimg->format.c_str(), matTypeToString(decoded.type()).c_str(),
                  timeToString(cimg->header.stamp).c_str());
      }
      return false;
    }
    return true;
  }

  if (shouldLogFailure(fail_counter)) {
    ROS_WARN("Depth message on topic '%s' has unsupported type '%s' at t=%s. Skipping.",
             mi.getTopic().c_str(), mi.getDataType().c_str(), timeToString(mi.getTime()).c_str());
  }
  return false;
}

// -----------------------------
// Confidence loading (NO conversion)
// -----------------------------
static bool loadConfidenceMatFromMI(const rosbag::MessageInstance& mi,
                                    cv::Mat& conf_out,
                                    std_msgs::Header& hdr,
                                    std::string& encoding_or_format_out,
                                    size_t& fail_counter) {
  conf_out.release();
  encoding_or_format_out.clear();

  if (auto img = mi.instantiate<sensor_msgs::Image>()) {
    hdr = img->header;
    encoding_or_format_out = img->encoding;

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(img, img->encoding);
    } catch (const cv_bridge::Exception& e) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("Confidence cv_bridge exception: %s (encoding='%s' stamp=%s)",
                  e.what(), img->encoding.c_str(), timeToString(img->header.stamp).c_str());
      }
      return false;
    }

    conf_out = cv_ptr->image.clone();
    return !conf_out.empty();
  }

  if (auto cimg = mi.instantiate<sensor_msgs::CompressedImage>()) {
    hdr = cimg->header;

    if (!decodeCompressedAnyToMat(*cimg, conf_out, encoding_or_format_out, fail_counter)) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("Failed to decode confidence CompressedImage (format='%s', bytes=%zu, stamp=%s).",
                  cimg->format.c_str(), cimg->data.size(), timeToString(cimg->header.stamp).c_str());
      }
      return false;
    }
    return !conf_out.empty();
  }

  if (shouldLogFailure(fail_counter)) {
    ROS_WARN("Confidence message on topic '%s' has unsupported type '%s' at t=%s. Skipping.",
             mi.getTopic().c_str(), mi.getDataType().c_str(), timeToString(mi.getTime()).c_str());
  }
  return false;
}

// -----------------------------
// Color decoding (BGR output)
// -----------------------------
static bool loadColorBgrFromMI(const rosbag::MessageInstance& mi,
                               cv::Mat& out_bgr,
                               std_msgs::Header& hdr,
                               std::string& msg_type,
                               std::string& enc_or_format,
                               std::string& enc_guess,
                               size_t& fail_counter) {
  if (auto img = mi.instantiate<sensor_msgs::Image>()) {
    hdr = img->header;
    msg_type = "sensor_msgs/Image";
    enc_or_format = img->encoding;
    enc_guess = guessRgbBgr(img->encoding);

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(img, "bgr8");
    } catch (const cv_bridge::Exception& e) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("Color cv_bridge exception: %s (encoding='%s' stamp=%s)",
                  e.what(), img->encoding.c_str(), timeToString(img->header.stamp).c_str());
      }
      return false;
    }

    out_bgr = cv_ptr->image;
    if (out_bgr.empty()) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("Color Image decoded to empty Mat (encoding='%s' stamp=%s)",
                  img->encoding.c_str(), timeToString(img->header.stamp).c_str());
      }
      return false;
    }
    return true;
  }

  if (auto cimg = mi.instantiate<sensor_msgs::CompressedImage>()) {
    hdr = cimg->header;
    msg_type = "sensor_msgs/CompressedImage";
    enc_or_format = cimg->format;
    enc_guess = guessRgbBgr(cimg->format);

    if (cimg->data.empty()) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("Color CompressedImage has empty data (format='%s' stamp=%s)",
                  cimg->format.c_str(), timeToString(cimg->header.stamp).c_str());
      }
      return false;
    }

    cv::Mat rawData(1, static_cast<int>(cimg->data.size()), CV_8UC1, (void*)cimg->data.data());
    out_bgr = cv::imdecode(rawData, cv::IMREAD_COLOR);
    if (out_bgr.empty()) {
      if (shouldLogFailure(fail_counter)) {
        ROS_ERROR("Failed to imdecode color CompressedImage (format='%s', bytes=%zu, stamp=%s).",
                  cimg->format.c_str(), cimg->data.size(), timeToString(cimg->header.stamp).c_str());
      }
      return false;
    }
    return true;
  }

  if (shouldLogFailure(fail_counter)) {
    ROS_WARN("Color message on topic '%s' has unsupported type '%s' at t=%s. Skipping.",
             mi.getTopic().c_str(), mi.getDataType().c_str(), timeToString(mi.getTime()).c_str());
  }
  return false;
}

// -----------------------------
// Output folder safety + cleanup
// -----------------------------


static bool findExactStampMatch(rosbag::View::const_iterator& it,
                                const rosbag::View::const_iterator& end,
                                const ros::Time& target,
                                rosbag::View::const_iterator& matched_out) {
  matched_out = end;
  while (it != end) {
    ros::Time ts;
    if (auto img = it->instantiate<sensor_msgs::Image>()) {
      ts = img->header.stamp;
    } else if (auto cimg = it->instantiate<sensor_msgs::CompressedImage>()) {
      ts = cimg->header.stamp;
    } else {
      ++it;
      continue;
    }

    if (ts < target) { ++it; continue; }
    if (ts == target) { matched_out = it; return true; }
    return false;  // ts > target
  }
  return false;
}

static void dumpTfFramesYaml(tf2_ros::Buffer& tf_buffer, size_t max_chars) {
  std::string y = tf_buffer.allFramesAsYAML();
  if (max_chars > 0 && y.size() > max_chars) {
    y = y.substr(0, max_chars);
    y += "\n... (truncated)\n";
  }
  std::cout << "\n" << ansi::kBold << ansi::kMagenta << "[TF_FRAMES_YAML]" << ansi::kReset << "\n"
            << y << "\n";
}

// -----------------------------
// Confidence saving helper
// -----------------------------
static bool saveConfidenceToDisk(const cv::Mat& conf_in,
                                 const std::string& out_path,
                                 bool saveAsEXR) {
  if (conf_in.empty()) return false;

  cv::Mat conf = conf_in;
  if (conf.channels() > 1) {
    cv::Mat gray;
    cv::cvtColor(conf, gray, cv::COLOR_BGR2GRAY);
    conf = gray;
  }

  if (saveAsEXR) {
    cv::Mat conf_f;
    if (conf.type() != CV_32FC1) {
      conf.convertTo(conf_f, CV_32FC1);
    } else {
      conf_f = conf;
    }
    return cv::imwrite(out_path, conf_f);
  }

  if (conf.type() == CV_8UC1 || conf.type() == CV_16UC1) {
    return cv::imwrite(out_path, conf);
  }

  cv::Mat conf_work = conf;
  if (conf_work.depth() == CV_32F) {
    cv::patchNaNs(conf_work, 0.0f);
  }

  double mn = 0.0, mx = 0.0;
  try {
    cv::minMaxLoc(conf_work, &mn, &mx);
  } catch (...) {
    mn = 0.0;
    mx = 0.0;
  }

  double scale = 1.0;
  if (mx <= 1.1 && mx >= 0.0) {
    scale = 65535.0;
  } else {
    scale = 1.0;
  }

  cv::Mat out16;
  conf_work.convertTo(out16, CV_16UC1, scale, 0.0);
  return cv::imwrite(out_path, out16);
}

// -----------------------------
// Trajectory helper (write 4x4 row-major)
// -----------------------------
static inline void writePoseRowMajor4x4(std::ofstream& ofs, const geometry_msgs::TransformStamped& tf) {
  tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                    tf.transform.rotation.z, tf.transform.rotation.w);
  tf2::Matrix3x3 Rm(q);

  const double r11 = Rm[0][0], r12 = Rm[0][1], r13 = Rm[0][2];
  const double r21 = Rm[1][0], r22 = Rm[1][1], r23 = Rm[1][2];
  const double r31 = Rm[2][0], r32 = Rm[2][1], r33 = Rm[2][2];

  const double px = tf.transform.translation.x;
  const double py = tf.transform.translation.y;
  const double pz = tf.transform.translation.z;

  double T[16];
  T[0] = r11; T[1] = r12; T[2] = r13; T[3] = px;
  T[4] = r21; T[5] = r22; T[6] = r23; T[7] = py;
  T[8] = r31; T[9] = r32; T[10] = r33; T[11] = pz;
  T[12] = 0;  T[13] = 0;  T[14] = 0;  T[15] = 1;

  for (int i = 0; i < 16; ++i) {
    ofs << T[i];
    if (i < 15) ofs << " ";
  }
  ofs << "\n";
}

// -----------------------------
// JSON helpers (CameraInfo export)
// -----------------------------
static inline std::string jsonEscape(const std::string& in) {
  std::ostringstream o;
  o << std::hex << std::setfill('0');
  for (unsigned char c : in) {
    switch (c) {
      case '\"': o << "\\\""; break;
      case '\\': o << "\\\\"; break;
      case '\b': o << "\\b"; break;
      case '\f': o << "\\f"; break;
      case '\n': o << "\\n"; break;
      case '\r': o << "\\r"; break;
      case '\t': o << "\\t"; break;
      default:
        if (c < 0x20) {
          o << "\\u" << std::setw(4) << static_cast<int>(c);
        } else {
          o << static_cast<char>(c);
        }
        break;
    }
  }
  return o.str();
}

static inline void jsonIndent(std::ofstream& ofs, int n) {
  for (int i = 0; i < n; ++i) ofs << ' ';
}

static void writeJsonArrayD(std::ofstream& ofs, const std::vector<double>& v, int indent) {
  ofs << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    ofs << std::setprecision(17) << v[i];
    if (i + 1 < v.size()) ofs << ", ";
  }
  ofs << "]";
  (void)indent;
}

static void writeJsonArrayFixed(std::ofstream& ofs, const double* data, size_t n) {
  ofs << "[";
  for (size_t i = 0; i < n; ++i) {
    ofs << std::setprecision(17) << data[i];
    if (i + 1 < n) ofs << ", ";
  }
  ofs << "]";
}

static void writeCameraInfoObject(std::ofstream& ofs,
                                  const sensor_msgs::CameraInfo& ci,
                                  int scale,
                                  int indent, bool rectified = true) {
  const int w = static_cast<int>(ci.width);
  const int h = static_cast<int>(ci.height);

  const double fx = rectified ? ci.P[0] : ci.K[0];
  const double fy = rectified ? ci.P[5] : ci.K[4];
  const double cx = rectified ? ci.P[2] : ci.K[2];
  const double cy = rectified ? ci.P[6] : ci.K[5];

  jsonIndent(ofs, indent); ofs << "{\n";

  jsonIndent(ofs, indent + 2); ofs << "\"rectified\": " << (rectified ? "true" : "false") << ",\n";
  // Header info
  jsonIndent(ofs, indent + 2); ofs << "\"frame_id\": \"" << jsonEscape(ci.header.frame_id) << "\",\n";
  jsonIndent(ofs, indent + 2); ofs << "\"stamp\": " << std::setprecision(17) << ci.header.stamp.toSec() << ",\n";

  // Basic intrinsics (legacy convenience fields)
  jsonIndent(ofs, indent + 2); ofs << "\"w\": " << w << ",\n";
  jsonIndent(ofs, indent + 2); ofs << "\"h\": " << h << ",\n";
  jsonIndent(ofs, indent + 2); ofs << "\"fx\": " << std::setprecision(17) << fx << ",\n";
  jsonIndent(ofs, indent + 2); ofs << "\"fy\": " << std::setprecision(17) << fy << ",\n";
  jsonIndent(ofs, indent + 2); ofs << "\"cx\": " << std::setprecision(17) << cx << ",\n";
  jsonIndent(ofs, indent + 2); ofs << "\"cy\": " << std::setprecision(17) << cy << ",\n";

  // Depth-scale convention used by the original exporter
  jsonIndent(ofs, indent + 2); ofs << "\"scale\": " << scale << ",\n";

  // Full CameraInfo fields
  jsonIndent(ofs, indent + 2); ofs << "\"distortion_model\": \"" << jsonEscape(ci.distortion_model) << "\",\n";

  jsonIndent(ofs, indent + 2); ofs << "\"D\": ";
  writeJsonArrayD(ofs, ci.D, indent + 2);
  ofs << ",\n";

  // K (3x3), R (3x3), P (3x4) in row-major order, as in the ROS message.
  jsonIndent(ofs, indent + 2); ofs << "\"K\": ";
  writeJsonArrayFixed(ofs, ci.K.data(), 9);
  ofs << ",\n";

  jsonIndent(ofs, indent + 2); ofs << "\"R\": ";
  writeJsonArrayFixed(ofs, ci.R.data(), 9);
  ofs << ",\n";

  jsonIndent(ofs, indent + 2); ofs << "\"P\": ";
  writeJsonArrayFixed(ofs, ci.P.data(), 12);
  ofs << ",\n";

  jsonIndent(ofs, indent + 2); ofs << "\"binning_x\": " << static_cast<unsigned int>(ci.binning_x) << ",\n";
  jsonIndent(ofs, indent + 2); ofs << "\"binning_y\": " << static_cast<unsigned int>(ci.binning_y) << ",\n";

  // ROI
  jsonIndent(ofs, indent + 2); ofs << "\"roi\": {\n";
  jsonIndent(ofs, indent + 4); ofs << "\"x_offset\": " << static_cast<unsigned int>(ci.roi.x_offset) << ",\n";
  jsonIndent(ofs, indent + 4); ofs << "\"y_offset\": " << static_cast<unsigned int>(ci.roi.y_offset) << ",\n";
  jsonIndent(ofs, indent + 4); ofs << "\"height\": " << static_cast<unsigned int>(ci.roi.height) << ",\n";
  jsonIndent(ofs, indent + 4); ofs << "\"width\": " << static_cast<unsigned int>(ci.roi.width) << ",\n";
  jsonIndent(ofs, indent + 4); ofs << "\"do_rectify\": " << (ci.roi.do_rectify ? "true" : "false") << "\n";
  jsonIndent(ofs, indent + 2); ofs << "}\n";

  jsonIndent(ofs, indent); ofs << "}";
}

// -----------------------------
// Main
// -----------------------------
static int processDataset(ros::NodeHandle& nh) {

  std::string global_rosbag_path;
  if (!nh.getParam("global_rosbag_path", global_rosbag_path)) {
    ROS_ERROR("Parameter 'global_rosbag_path' not set!");
    return -1;
  }

  // OPTIONAL: empty => disable
  std::string depth_topic;
  nh.param("depth_topic", depth_topic, std::string(""));

  // OPTIONAL: empty => disable confidence
  std::string confidence_topic;
  nh.param("confidence_topic", confidence_topic, std::string(""));

  // OPTIONAL: default false
  bool save_confidence = false;
  nh.param("save_confidence", save_confidence, false);
  if (save_confidence && confidence_topic.empty()) {
    std::cout << ansi::kBold << ansi::kYellow << "[WARN]" << ansi::kReset << " "
              << "save_confidence=true but confidence_topic is empty. Disabling confidence.\n";
    throw std::runtime_error("save_confidence requires an explicit confidence topic");
  }

  // Back-compat: color_topic -> right_color_topic
  std::string right_color_topic;
  if (!nh.getParam("right_color_topic", right_color_topic)) {
    std::string color_topic;
    if (!nh.getParam("color_topic", color_topic)) {
      ROS_ERROR("Parameter 'right_color_topic' (or legacy 'color_topic') not set!");
      return -1;
    }
    right_color_topic = color_topic;
  }

  // OPTIONAL: empty => MONO
  std::string left_color_topic;
  nh.param("left_color_topic", left_color_topic, std::string(""));

  // Back-compat: camera_info_topic -> right_camera_info_topic
  std::string right_camera_info_topic;
  if (!nh.getParam("right_camera_info_topic", right_camera_info_topic)) {
    std::string camera_info_topic;
    if (!nh.getParam("camera_info_topic", camera_info_topic)) {
      ROS_ERROR("Parameter 'right_camera_info_topic' (or legacy 'camera_info_topic') not set!");
      return -1;
    }
    right_camera_info_topic = camera_info_topic;
  }

  std::string left_camera_info_topic;
  nh.getParam("left_camera_info_topic", left_camera_info_topic);  // optional

  std::string bag_file_directory;
  if (!nh.getParam("bag_file_directory", bag_file_directory)) {
    ROS_ERROR("Parameter 'bag_file_directory' not set!");
    return -1;
  }

  std::string depth_bag_match_substring = "_depth.bag";
  std::string rgb_bag_match_substring   = "_images.bag";
  nh.getParam("depth_bag_match_substring", depth_bag_match_substring);
  nh.getParam("rgb_bag_match_substring", rgb_bag_match_substring);

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

  std::string folder_path;
  if (!nh.getParam("output_folder_path", folder_path)) {
    ROS_ERROR("Parameter 'output_folder_path' not set!");
    return -1;
  }

  bool print_tf_frames = false;
  nh.param("print_tf_frames", print_tf_frames, false);
  int tf_dump_max_chars_i = 8000;
  nh.param("tf_dump_max_chars", tf_dump_max_chars_i, 8000);
  const size_t tf_dump_max_chars = (tf_dump_max_chars_i < 0) ? 0 : static_cast<size_t>(tf_dump_max_chars_i);

  box_export::AtomicDirectory dataset(folder_path);
  fs::path out_base = dataset.path();
  bool images_rectified = true;
  nh.param("images_rectified", images_rectified, true);
  if (frames_to_skip < 0 || max_color_images <= 0 || max_depth_images <= 0)
    throw std::runtime_error("Frame limits must be positive; frames_to_skip must be nonnegative");
  std::ofstream manifest(out_base / "frames.csv");
  manifest.exceptions(std::ios::failbit | std::ios::badbit);
  manifest << "stream,index,stamp_ns,frame_id,file,width,height\n";
  std::map<std::string, uint64_t> last_stamp;
  std::map<std::string, std::pair<int, int>> dimensions;
  std::map<std::string, std::string> optical_frames;
  std::map<std::string, size_t> frame_counts;
  auto recordFrame = [&](const std::string& stream, int index, const std_msgs::Header& header,
                         const fs::path& file, const cv::Mat& pixels) {
    const auto stamp = header.stamp.toNSec();
    if (!stamp || header.frame_id.empty() || header.frame_id.find_first_of(",\"\n\r") != std::string::npos ||
        (last_stamp.count(stream) && stamp <= last_stamp.at(stream)))
      throw std::runtime_error("Invalid or non-increasing acquisition metadata in " + stream);
    const auto size = std::make_pair(pixels.cols, pixels.rows);
    if (dimensions.count(stream) && (dimensions.at(stream) != size || optical_frames.at(stream) != header.frame_id))
      throw std::runtime_error("Image dimensions or optical frame changed in " + stream);
    dimensions[stream] = size;
    optical_frames[stream] = header.frame_id;
    last_stamp[stream] = stamp;
    ++frame_counts[stream];
    manifest << stream << ',' << index << ',' << stamp << ',' << header.frame_id << ','
             << fs::relative(file, out_base).generic_string() << ',' << pixels.cols << ',' << pixels.rows << '\n';
  };

  std::string map_frame = "dlio_map";
  if (!nh.getParam("map_frame", map_frame)) {
    ROS_ERROR("Parameter 'map_frame' not set!");
    return -1;
  }

  // Resolve bag paths
  std::string rgbBagPath, depthBagPath;
  bool have_depth_bag = true;

  if (!global_rosbag_path.empty()) {
    rgbBagPath = global_rosbag_path;
    depthBagPath = global_rosbag_path;
  } else {
    // Depth bag optional
    try {
      if (!depth_bag_match_substring.empty()) {
        if (findSingleBagBySubstring(bag_file_directory, depth_bag_match_substring, depthBagPath)) {
          std::cout << "Matched depth bag: " << depthBagPath << std::endl;
        } else {
          have_depth_bag = false;
          std::cout << ansi::kBold << ansi::kYellow << "[WARN]" << ansi::kReset << " "
                    << "No file containing '" << depth_bag_match_substring << "' found in directory "
                    << ansi::kCyan << bag_file_directory << ansi::kReset
                    << ". Continuing WITHOUT depth/confidence.\n";
        }
      } else {
        have_depth_bag = false;
        std::cout << ansi::kBold << ansi::kYellow << "[WARN]" << ansi::kReset << " "
                  << "depth_bag_match_substring is empty. Depth/confidence disabled.\n";
      }
    } catch (const std::exception& e) {
      ROS_ERROR("Depth bag search failed: %s", e.what());
      return -1;
    }

    // RGB bag required
    try {
      if (!findSingleBagBySubstring(bag_file_directory, rgb_bag_match_substring, rgbBagPath)) {
        ROS_ERROR("No files containing '%s' found in directory '%s'.",
                  rgb_bag_match_substring.c_str(), bag_file_directory.c_str());
        return -1;
      }
      std::cout << "Matched RGB bag: " << rgbBagPath << std::endl;
    } catch (const std::exception& e) {
      ROS_ERROR("RGB bag search failed: %s", e.what());
      return -1;
    }
  }

  // Open bags
  rosbag::Bag depthBag, rgbBag;
  bool depthBagOpened = false;

  try {
    ROS_INFO("Opening RGB bag file: %s", rgbBagPath.c_str());
    rgbBag.open(rgbBagPath, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& e) {
    ROS_ERROR("Failed to open RGB bag file: %s", e.what());
    return -1;
  }

  if (!depthBagPath.empty() && have_depth_bag) {
    try {
      ROS_INFO("Opening depth bag file: %s", depthBagPath.c_str());
      depthBag.open(depthBagPath, rosbag::bagmode::Read);
      depthBagOpened = true;
    } catch (const rosbag::BagException& e) {
      have_depth_bag = false;
      depthBagOpened = false;
      std::cout << ansi::kBold << ansi::kYellow << "[WARN]" << ansi::kReset << " "
                << "Failed to open depth bag '" << ansi::kCyan << depthBagPath << ansi::kReset
                << "': " << e.what()
                << ". Continuing WITHOUT depth/confidence.\n";
    }
  }

  // Decide mono vs stereo (robust to "left topic set but not present").
  rosbag::View right_view(rgbBag, rosbag::TopicQuery(right_color_topic));
  const size_t total_right_msgs = right_view.size();
  if (total_right_msgs == 0) {
    ROS_ERROR("Right color topic '%s' has 0 messages in RGB bag '%s'.",
              right_color_topic.c_str(), rgbBagPath.c_str());
    return -1;
  }

  bool stereo_enabled = !left_color_topic.empty();
  std::unique_ptr<rosbag::View> left_view_ptr;
  size_t total_left_msgs = 0;
  if (stereo_enabled) {
    left_view_ptr = std::make_unique<rosbag::View>(rgbBag, rosbag::TopicQuery(left_color_topic));
    total_left_msgs = left_view_ptr->size();
    if (total_left_msgs == 0) {
      std::cout << ansi::kBold << ansi::kYellow << "[WARN]" << ansi::kReset << " "
                << "left_color_topic='" << ansi::kCyan << left_color_topic << ansi::kReset
                << "' has 0 messages. Falling back to MONO export.\n";
      throw std::runtime_error("Requested left image topic is empty");
    }
  }

  // Depth/conf optional: requires usable depth bag + non-empty topic.
  bool depth_enabled = (have_depth_bag && depthBagOpened && !depth_topic.empty());
  if (!depth_enabled) {
    if (!depth_topic.empty() && !(have_depth_bag && depthBagOpened)) {
      std::cout << ansi::kBold << ansi::kYellow << "[WARN]" << ansi::kReset << " "
                << "depth_topic is set but no usable depth bag is available. Depth/confidence disabled.\n";
    }
  }

  if ((!depth_topic.empty() && !depth_enabled) || (save_confidence && !depth_enabled))
    throw std::runtime_error("Requested depth/confidence input is unavailable");

  // Output folders (created based on mono/stereo + optional depth/conf).
  fs::path image_folder_mono   = out_base / "image";
  fs::path images_folder       = out_base / "images";
  fs::path right_images_folder = images_folder / "right";
  fs::path left_images_folder  = images_folder / "left";
  fs::path depth_images_folder = out_base / "depth_images";
  fs::path confidence_folder   = out_base / "confidence_images";

  try {
    if (stereo_enabled) {
      fs::create_directories(right_images_folder);
      fs::create_directories(left_images_folder);
      ROS_INFO("Right images folder: %s", right_images_folder.string().c_str());
      ROS_INFO("Left images folder:  %s", left_images_folder.string().c_str());
    } else {
      fs::create_directories(image_folder_mono);
      ROS_INFO("Image folder:        %s", image_folder_mono.string().c_str());
    }

    if (depth_enabled) {
      fs::create_directories(depth_images_folder);
      ROS_INFO("Depth images folder: %s", depth_images_folder.string().c_str());
      if (save_confidence) {
        fs::create_directories(confidence_folder);
        ROS_INFO("Confidence folder:   %s", confidence_folder.string().c_str());
      }
    }
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to create output subdirectories: %s", e.what());
    return -1;
  }

  // TF bag list (absolute param name expected)
  std::vector<std::string> tfContainingBags;
  if (!nh.getParam("tf_containing_bag_paths", tfContainingBags))
    nh.getParam("/tf_containing_bag_paths", tfContainingBags);

  if (tfContainingBags.empty()) {
    ROS_ERROR_STREAM("No TF Input bag is provided.");
    ros::shutdown();
    return -1;
  }

  ROS_WARN("Reading full RGB bag file to get duration for TF buffer");
  rosbag::View full_view(rgbBag);
  const ros::Time rgb_start_time = full_view.getBeginTime();
  const ros::Time rgb_end_time = full_view.getEndTime();
  const ros::Duration rgb_duration = rgb_end_time - rgb_start_time;

  const double cache_margin_s = 5.0;
  tf2_ros::Buffer tf_buffer(ros::Duration(std::max(0.0, rgb_duration.toSec() + cache_margin_s)));

  const double tf_timeout_seconds = 1.0;
  const ros::Time tf_min_stamp = rgb_start_time - ros::Duration(cache_margin_s);
  const ros::Time tf_max_stamp = rgb_end_time + ros::Duration(tf_timeout_seconds);

  // ---------------- Depth saving ----------------
  if (depth_enabled) {
    ROS_WARN("Getting depth (supports Image, CompressedImage, and compressedDepth png/rvl)");
    rosbag::View depth_view(depthBag, rosbag::TopicQuery(depth_topic));
    const size_t total_depth_msgs = depth_view.size();

    if (total_depth_msgs == 0) {
      std::cout << ansi::kBold << ansi::kYellow << "[WARN]" << ansi::kReset << " "
                << "Depth bag is open but topic '" << ansi::kCyan << depth_topic << ansi::kReset
                << "' has 0 messages. Continuing WITHOUT depth/confidence.\n";
      throw std::runtime_error("Requested depth topic is empty");
    } else {
      int saved_depth_count = 0;
      int nbDepthFrames = 0;

      bool printed_depth_stream_header = false;
      size_t depth_seen = 0;
      size_t depth_failures = 0;

      foreach (rosbag::MessageInstance const mi, depth_view) {
        if (saved_depth_count >= max_depth_images) break;
        ++depth_seen;
        printProgressBar("Depth     ", depth_seen, total_depth_msgs);

        cv::Mat depth_meters;
        std_msgs::Header hdr;

        if (!printed_depth_stream_header) {
          if (auto img = mi.instantiate<sensor_msgs::Image>()) {
            logStreamHeaderOnce("DEPTH", depth_topic, "sensor_msgs/Image", img->encoding, img->header.frame_id);
          } else if (auto cimg = mi.instantiate<sensor_msgs::CompressedImage>()) {
            logStreamHeaderOnce("DEPTH", depth_topic, "sensor_msgs/CompressedImage", cimg->format, cimg->header.frame_id);
          } else {
            logStreamHeaderOnce("DEPTH", depth_topic, "<unknown>", mi.getDataType(), "<unknown>");
          }
          printed_depth_stream_header = true;
        }

        if (!loadDepthMetersFromMI(mi, depth_meters, hdr, depth_failures))
          throw std::runtime_error("Failed to decode selected depth image");

        ++nbDepthFrames;
        if (nbDepthFrames <= frames_to_skip) continue;

        if (saveAsEXR) {
          std::ostringstream filename;
          filename << "depth" << saved_depth_count << ".exr";
          const std::string out_path = (depth_images_folder / filename.str()).string();
          if (!cv::imwrite(out_path, depth_meters)) {
            ROS_ERROR("Failed to write %s", out_path.c_str());
            throw std::runtime_error("Failed to write depth image");
          }
        } else {
          cv::Mat depth_16u;
          cv::Mat png_depth = depth_meters.clone();
          for (int row = 0; row < png_depth.rows; ++row)
            for (int col = 0; col < png_depth.cols; ++col) {
              float& value = png_depth.at<float>(row, col);
              if (std::isfinite(value) && value > 65.535f)
                throw std::runtime_error("Depth exceeds uint16 millimetres; select save_as_exr=true");
              if (!std::isfinite(value) || value <= 0) value = 0;  // PNG convention: 0 is invalid.
            }
          png_depth.convertTo(depth_16u, CV_16UC1, 1000.0);
          std::ostringstream filename;
          filename << "depth" << saved_depth_count << ".png";
          const std::string out_path = (depth_images_folder / filename.str()).string();
          if (!cv::imwrite(out_path, depth_16u)) {
            ROS_ERROR("Failed to write %s", out_path.c_str());
            throw std::runtime_error("Failed to write depth image");
          }
        }
        recordFrame("depth", saved_depth_count, hdr, depth_images_folder / ("depth" + std::to_string(saved_depth_count) + (saveAsEXR ? ".exr" : ".png")), depth_meters);

        ROS_INFO("Saved depth%d.%s", saved_depth_count, saveAsEXR ? "exr" : "png");
        ++saved_depth_count;
      }
      printProgressBar("Depth     ", total_depth_msgs, total_depth_msgs);
    }
  }

  // ---------------- Confidence saving ----------------
  if (save_confidence) {
    if (!depth_enabled) {
      std::cout << ansi::kBold << ansi::kYellow << "[WARN]" << ansi::kReset << " "
                << "save_confidence=true but depth is disabled/unavailable; skipping confidence.\n";
    } else {
      std::vector<std::string> conf_topics_to_try;
      conf_topics_to_try.push_back(confidence_topic);
      if (confidence_topic.find("compressedDepth") != std::string::npos) {
        conf_topics_to_try.push_back(std::regex_replace(confidence_topic, std::regex("compressedDepth"), "compressed"));
      } else if (confidence_topic.find("/compressed") != std::string::npos) {
        conf_topics_to_try.push_back(std::regex_replace(confidence_topic, std::regex("/compressed$"), "/compressedDepth"));
      }

      // FIX: choose the topic first (rosbag::View is non-assignable), then construct the View once.
      std::string used_conf_topic;
      size_t total_conf_msgs = 0;
      for (const auto& t : conf_topics_to_try) {
        rosbag::View tmp(depthBag, rosbag::TopicQuery(t));
        if (tmp.size() > 0) {
          used_conf_topic = t;
          total_conf_msgs = tmp.size();
          break;
        }
      }

      if (total_conf_msgs == 0) {
        std::cout << ansi::kBold << ansi::kYellow << "[WARN]" << ansi::kReset << " "
                  << "Confidence topic '" << ansi::kCyan << confidence_topic << ansi::kReset
                  << "' has 0 messages (and fallbacks empty). Skipping confidence.\n";
      } else {
        if (used_conf_topic != confidence_topic) {
          std::cout << ansi::kBold << ansi::kYellow << "[WARN]" << ansi::kReset << " "
                    << "Confidence topic '" << ansi::kCyan << confidence_topic << ansi::kReset
                    << "' had 0 messages; using fallback topic '"
                    << ansi::kCyan << used_conf_topic << ansi::kReset << "'.\n";
        }

        rosbag::View confidence_view(depthBag, rosbag::TopicQuery(used_conf_topic));

        int saved_conf_count = 0;
        int nbConfFrames = 0;
        size_t conf_seen = 0;
        size_t conf_failures = 0;
        bool printed_conf_stream_header = false;

        foreach (rosbag::MessageInstance const mi, confidence_view) {
          if (saved_conf_count >= max_depth_images) break;
          ++conf_seen;
          printProgressBar("Confidence", conf_seen, total_conf_msgs);

          cv::Mat conf_mat;
          std_msgs::Header hdr;
          std::string encfmt;
          if (!loadConfidenceMatFromMI(mi, conf_mat, hdr, encfmt, conf_failures))
            throw std::runtime_error("Failed to decode selected confidence image");

          if (!printed_conf_stream_header) {
            if (auto img = mi.instantiate<sensor_msgs::Image>()) {
              logStreamHeaderOnce("CONF", used_conf_topic, "sensor_msgs/Image", img->encoding, img->header.frame_id);
            } else if (auto cimg = mi.instantiate<sensor_msgs::CompressedImage>()) {
              logStreamHeaderOnce("CONF", used_conf_topic, "sensor_msgs/CompressedImage", cimg->format, cimg->header.frame_id);
            } else {
              logStreamHeaderOnce("CONF", used_conf_topic, "<unknown>", mi.getDataType(), "<unknown>");
            }
            printed_conf_stream_header = true;
          }

          ++nbConfFrames;
          if (nbConfFrames <= frames_to_skip) continue;

          std::ostringstream filename;
          filename << "confidence" << saved_conf_count << "." << (saveAsEXR ? "exr" : "png");
          const std::string out_path = (confidence_folder / filename.str()).string();

          if (!saveConfidenceToDisk(conf_mat, out_path, saveAsEXR)) {
            ROS_ERROR("Failed to write %s (conf type=%s enc/format=%s)",
                      out_path.c_str(), matTypeToString(conf_mat.type()).c_str(), encfmt.c_str());
            throw std::runtime_error("Failed to write confidence image");
          }

          recordFrame("confidence", saved_conf_count, hdr, out_path, conf_mat);
          ++saved_conf_count;
        }
        printProgressBar("Confidence", total_conf_msgs, total_conf_msgs);
      }
    }
  }

  // ---------------- TF accumulation (FIXED) ----------------
  std::vector<rosbag::Bag> bags;
  bags.reserve(tfContainingBags.size());

  std::cout << ansi::kBold << ansi::kGreen << "[TF] " << ansi::kReset
            << "tf_containing_bag_paths size=" << tfContainingBags.size() << "\n";

  for (const auto& path_str : tfContainingBags) {
    fs::path p(path_str);
    fs::path candidate = p;

    if (!p.is_absolute()) {
      if (!bag_file_directory.empty()) {
        candidate = fs::path(bag_file_directory) / p;
      }
    }

    std::cout << ansi::kGreen << "[TF] " << ansi::kReset
              << "Opening bag: " << ansi::kCyan << candidate.string() << ansi::kReset << "\n";

    bags.emplace_back();
    try {
      bags.back().open(candidate.string(), rosbag::bagmode::Read);
    } catch (const std::exception& e1) {
      if (candidate.string() != path_str) {
        std::cout << ansi::kBold << ansi::kYellow << "[WARN]" << ansi::kReset << " "
                  << "Failed to open TF bag '" << candidate.string() << "': " << e1.what()
                  << ". Trying original path '" << path_str << "'.\n";
        try {
          bags.back().open(path_str, rosbag::bagmode::Read);
        } catch (const std::exception& e2) {
          ROS_ERROR("Failed to open TF bag '%s' and '%s': %s",
                    candidate.string().c_str(), path_str.c_str(), e2.what());
          return -1;
        }
      } else {
        ROS_ERROR("Failed to open TF bag '%s': %s", candidate.string().c_str(), e1.what());
        return -1;
      }
    }
  }

  std::vector<std::string> tf_topics{"/tf", "/tf_static"};
  rosbag::View combined_view;
  for (auto& tfbag : bags) {
    combined_view.addQuery(tfbag, rosbag::TopicQuery(tf_topics));
  }

  const size_t total_tf_msgs = combined_view.size();
  if (total_tf_msgs == 0) {
    ROS_WARN("No TF messages found in provided TF bags on topics /tf or /tf_static.");
  }

  size_t tf_seen = 0;
  size_t tf_kept = 0;
  size_t tf_skipped_outside_window = 0;
  size_t tf_set_fail = 0;
  size_t tf_static_set = 0;

  for (const rosbag::MessageInstance& m : combined_view) {
    ++tf_seen;
    printProgressBar("TF        ", tf_seen, total_tf_msgs);

    const std::string& topic_name = m.getTopic();
    const bool is_tf = (topic_name == "/tf" || topic_name == "tf");
    const bool is_tf_static = (topic_name == "/tf_static" || topic_name == "tf_static");

    tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
    if (!tf_msg) continue;

    if (is_tf_static) {
      for (const geometry_msgs::TransformStamped& tr : tf_msg->transforms) {
        try {
          tf_buffer.setTransform(tr, "default_authority", true);
          ++tf_static_set;
        } catch (const std::exception&) {
          ++tf_set_fail;
        }
      }
      continue;
    }

    if (is_tf) {
      for (const geometry_msgs::TransformStamped& tr : tf_msg->transforms) {
        ros::Time ts = tr.header.stamp;
        if (ts.isZero()) ts = m.getTime();  // fallback only

        if (ts < tf_min_stamp || ts > tf_max_stamp) {
          ++tf_skipped_outside_window;
          continue;
        }

        try {
          tf_buffer.setTransform(tr, "default_authority", false);
          ++tf_kept;
        } catch (const std::exception&) {
          ++tf_set_fail;
        }
      }
    }
  }
  printProgressBar("TF        ", total_tf_msgs, total_tf_msgs);

  for (auto& bag : bags) {
    if (bag.isOpen()) bag.close();
  }

  std::cout << ansi::kBold << ansi::kGreen << "[TF] " << ansi::kReset
            << "Loaded tf_static=" << tf_static_set
            << " tf_kept=" << tf_kept
            << " tf_skipped_outside_window=" << tf_skipped_outside_window
            << " tf_set_fail=" << tf_set_fail
            << " window=[" << timeToString(tf_min_stamp) << ", " << timeToString(tf_max_stamp) << "]\n";

  if (print_tf_frames) {
    dumpTfFramesYaml(tf_buffer, tf_dump_max_chars);
  }

  // ---------------- RGB saving + trajectories (MONO or STEREO) ----------------
  if (stereo_enabled) {
    ROS_WARN("Getting RIGHT+LEFT color (paired by exact header.stamp)");
    rosbag::View& left_view = *left_view_ptr;

    auto left_it  = left_view.begin();
    auto left_end = left_view.end();

    int saved_pair_count = 0;
    int nbRightFrames = 0;

    bool printed_right_stream_header = false;
    bool printed_left_stream_header  = false;

    size_t right_seen = 0;
    size_t right_failures = 0;
    size_t left_failures  = 0;

    const std::string traj_right_path = (out_base / "traj_right.txt").string();
    const std::string traj_left_path  = (out_base / "traj_left.txt").string();

    std::ofstream ofs_right(traj_right_path);
    ofs_right.exceptions(std::ios::failbit | std::ios::badbit);
    if (!ofs_right.is_open()) {
      ROS_ERROR("Failed to open output file: %s", traj_right_path.c_str());
      return -1;
    }
    std::ofstream ofs_left(traj_left_path);
    ofs_left.exceptions(std::ios::failbit | std::ios::badbit);
    if (!ofs_left.is_open()) {
      ROS_ERROR("Failed to open output file: %s", traj_left_path.c_str());
      return -1;
    }

    for (const rosbag::MessageInstance& m_right : right_view) {
      if (saved_pair_count >= max_color_images) break;

      ++right_seen;
      printProgressBar("Right     ", right_seen, total_right_msgs);

      cv::Mat right_bgr;
      std_msgs::Header right_hdr;
      std::string right_msg_type, right_encfmt, right_guess;

      if (!loadColorBgrFromMI(m_right, right_bgr, right_hdr, right_msg_type, right_encfmt, right_guess, right_failures))
        throw std::runtime_error("Failed to decode selected right image");

      if (!printed_right_stream_header) {
        logStreamHeaderOnce("RIGHT", right_color_topic, right_msg_type, right_encfmt, right_hdr.frame_id);
        printed_right_stream_header = true;
      }

      ++nbRightFrames;
      if (nbRightFrames <= frames_to_skip) continue;

      rosbag::View::const_iterator matched_left_it = left_end;
      if (!findExactStampMatch(left_it, left_end, right_hdr.stamp, matched_left_it))
        throw std::runtime_error("Missing exact left/right timestamp match");

      cv::Mat left_bgr;
      std_msgs::Header left_hdr;
      std::string left_msg_type, left_encfmt, left_guess;

      if (!loadColorBgrFromMI(*matched_left_it, left_bgr, left_hdr, left_msg_type, left_encfmt, left_guess, left_failures)) {
        throw std::runtime_error("Failed to decode selected left image");
      }

      if (!printed_left_stream_header) {
        logStreamHeaderOnce("LEFT", left_color_topic, left_msg_type, left_encfmt, left_hdr.frame_id);
        printed_left_stream_header = true;
      }

      std::ostringstream fname;
      fname << "frame" << saved_pair_count << ".jpg";
      const std::string out_right = (right_images_folder / fname.str()).string();
      const std::string out_left  = (left_images_folder  / fname.str()).string();

      if (!cv::imwrite(out_right, right_bgr)) {
        ROS_ERROR("Failed to write %s", out_right.c_str());
        return -1;
      }
      if (!cv::imwrite(out_left, left_bgr)) {
        ROS_ERROR("Failed to write %s", out_left.c_str());
        return -1;
      }

      try {
        geometry_msgs::TransformStamped tf_right =
            tf_buffer.lookupTransform(map_frame, right_hdr.frame_id, right_hdr.stamp);
        writePoseRowMajor4x4(ofs_right, tf_right);
      } catch (const std::exception& e) {
        ROS_ERROR("TF lookupTransform failed for RIGHT stamp=%s frame_id='%s' map_frame='%s': %s",
                  timeToString(right_hdr.stamp).c_str(),
                  right_hdr.frame_id.c_str(),
                  map_frame.c_str(),
                  e.what());
        dumpTfFramesYaml(tf_buffer, tf_dump_max_chars);
        return -1;
      }

      try {
        geometry_msgs::TransformStamped tf_left =
            tf_buffer.lookupTransform(map_frame, left_hdr.frame_id, left_hdr.stamp);
        writePoseRowMajor4x4(ofs_left, tf_left);
      } catch (const std::exception& e) {
        ROS_ERROR("TF lookupTransform failed for LEFT stamp=%s frame_id='%s' map_frame='%s': %s",
                  timeToString(left_hdr.stamp).c_str(),
                  left_hdr.frame_id.c_str(),
                  map_frame.c_str(),
                  e.what());
        dumpTfFramesYaml(tf_buffer, tf_dump_max_chars);
        return -1;
      }

      if (matched_left_it != left_end && left_it == matched_left_it) ++left_it;
      recordFrame("right", saved_pair_count, right_hdr, out_right, right_bgr);
      recordFrame("left", saved_pair_count, left_hdr, out_left, left_bgr);
      ++saved_pair_count;
    }
    printProgressBar("Right     ", total_right_msgs, total_right_msgs);

    ofs_right.close();
    ofs_left.close();

    if (saved_pair_count == 0) {
      ROS_ERROR("No paired right+left frames saved. Check frames_to_skip and stamp alignment.");
      return -1;
    }
  } else {
    ROS_WARN("Getting MONO color (single RGB stream)");

    int saved_count = 0;
    int nbFrames = 0;
    size_t seen = 0;
    size_t failures = 0;
    bool printed_stream_header = false;

    const std::string traj_path = (out_base / "traj.txt").string();
    std::ofstream ofs(traj_path);
    ofs.exceptions(std::ios::failbit | std::ios::badbit);
    if (!ofs.is_open()) {
      ROS_ERROR("Failed to open output file: %s", traj_path.c_str());
      return -1;
    }

    for (const rosbag::MessageInstance& m : right_view) {
      if (saved_count >= max_color_images) break;

      ++seen;
      printProgressBar("Image     ", seen, total_right_msgs);

      cv::Mat bgr;
      std_msgs::Header hdr;
      std::string msg_type, encfmt, guess;
      if (!loadColorBgrFromMI(m, bgr, hdr, msg_type, encfmt, guess, failures))
        throw std::runtime_error("Failed to decode selected color image");

      if (!printed_stream_header) {
        logStreamHeaderOnce("IMAGE", right_color_topic, msg_type, encfmt, hdr.frame_id);
        printed_stream_header = true;
      }

      ++nbFrames;
      if (nbFrames <= frames_to_skip) continue;

      std::ostringstream fname;
      fname << "frame" << saved_count << ".jpg";
      const std::string out_path = (image_folder_mono / fname.str()).string();
      if (!cv::imwrite(out_path, bgr)) {
        ROS_ERROR("Failed to write %s", out_path.c_str());
        return -1;
      }

      try {
        geometry_msgs::TransformStamped tf_cam =
            tf_buffer.lookupTransform(map_frame, hdr.frame_id, hdr.stamp);
        writePoseRowMajor4x4(ofs, tf_cam);
      } catch (const std::exception& e) {
        ROS_ERROR("TF lookupTransform failed for MONO stamp=%s frame_id='%s' map_frame='%s': %s",
                  timeToString(hdr.stamp).c_str(),
                  hdr.frame_id.c_str(),
                  map_frame.c_str(),
                  e.what());
        dumpTfFramesYaml(tf_buffer, tf_dump_max_chars);
        return -1;
      }

      recordFrame("right", saved_count, hdr, out_path, bgr);
      ++saved_count;
    }
    printProgressBar("Image     ", total_right_msgs, total_right_msgs);
    ofs.close();

    if (saved_count == 0) {
      ROS_ERROR("No mono frames saved. Check frames_to_skip and topic/encoding.");
      return -1;
    }
  }

  // ---------------- Camera info JSON (RIGHT + optional LEFT) ----------------
  auto readCalibration = [](rosbag::Bag& bag, const std::string& topic) {
    sensor_msgs::CameraInfo::ConstPtr selected;
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    for (const auto& message : view) {
      auto info = message.instantiate<sensor_msgs::CameraInfo>();
      if (!info) throw std::runtime_error("Expected CameraInfo on " + topic);
      if (selected && (selected->K != info->K || selected->D != info->D || selected->R != info->R ||
                       selected->P != info->P || selected->width != info->width || selected->height != info->height ||
                       selected->header.frame_id != info->header.frame_id))
        throw std::runtime_error("Calibration changes within the export: " + topic);
      selected = info;
    }
    return selected;
  };
  auto right_ci = readCalibration(rgbBag, right_camera_info_topic);
  auto left_ci = left_camera_info_topic.empty() ? sensor_msgs::CameraInfo::ConstPtr() : readCalibration(rgbBag, left_camera_info_topic);
  sensor_msgs::CameraInfo::ConstPtr depth_ci;
  if (depth_enabled) {
    std::string depth_info_topic;
    nh.param("depth_camera_info_topic", depth_info_topic, depth_topic.substr(0, depth_topic.find("/image")) + "/camera_info");
    depth_ci = readCalibration(depthBag, depth_info_topic);
  }
  auto validateCalibration = [&](const std::string& stream, sensor_msgs::CameraInfo::ConstPtr info) {
    if (!info || !frame_counts[stream] ||
        dimensions.at(stream) != std::make_pair(static_cast<int>(info->width), static_cast<int>(info->height)) ||
        optical_frames.at(stream) != info->header.frame_id)
      throw std::runtime_error("Missing/inconsistent calibration or no exported frames for " + stream);
    const auto finite = [](double value) { return std::isfinite(value); };
    if (!std::all_of(info->K.begin(), info->K.end(), finite) || !std::all_of(info->P.begin(), info->P.end(), finite) ||
        !std::all_of(info->R.begin(), info->R.end(), finite) || !std::all_of(info->D.begin(), info->D.end(), finite) ||
        info->K[0] <= 0 || info->K[4] <= 0 || info->P[0] <= 0 || info->P[5] <= 0)
      throw std::runtime_error("Invalid calibration for " + stream);
  };
  validateCalibration("right", right_ci);
  if (stereo_enabled) validateCalibration("left", left_ci);
  if (depth_enabled) validateCalibration("depth", depth_ci);
  if (save_confidence) validateCalibration("confidence", depth_ci);
  const int scale = saveAsEXR ? 1 : 1000;
  std::ofstream jofs(out_base / "camera_info.json");
  jofs.exceptions(std::ios::failbit | std::ios::badbit);
  jofs << "{\n  \"depth_scale\": " << scale << ",\n  \"right\": ";
  writeCameraInfoObject(jofs, *right_ci, scale, 2, images_rectified);
  if (left_ci) {
    jofs << ",\n  \"left\": ";
    writeCameraInfoObject(jofs, *left_ci, scale, 2, images_rectified);
  }
  if (depth_ci) {
    jofs << ",\n  \"depth\": ";
    writeCameraInfoObject(jofs, *depth_ci, scale, 2, true);
  }
  jofs << "\n}\n";
  jofs.close();
  manifest.close();

  rgbBag.close();
  if (depthBagOpened) depthBag.close();

  dataset.commit();
  ROS_WARN("Completed successfully. Self-terminating.");
  ros::shutdown();
  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_and_depth_save_node");
  ros::NodeHandle nh("~");
  try { return processDataset(nh); }
  catch (const std::exception& error) {
    ROS_ERROR_STREAM("Export failed: " << error.what());
    return 1;
  }
}

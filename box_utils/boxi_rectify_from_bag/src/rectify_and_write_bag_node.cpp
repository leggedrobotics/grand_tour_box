// Export synchronized stereo/depth frames using the calibration carried by each
// stream. SDK-rectified input is passed through, avoiding a second rectification.
#include <rectify_from_bag/rectify_from_bag.hpp>
#include <rectify_from_bag/turbojpeg_utils.hpp>
#include <box_export/atomic_directory.hpp>

#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>

namespace fs = std::filesystem;
using Index = std::map<uint64_t, rosbag::MessageInstance>;

static std::string parameter(const ros::NodeHandle& nh, const std::string& name,
                             const std::string& fallback = "") {
  std::string value;
  nh.param(name, value, fallback);
  return value;
}

static std::string resolveBag(const ros::NodeHandle& nh, const std::string& side,
                              const std::string& fallback = "") {
  const auto explicit_path = parameter(nh, side + "_bag");
  if (!explicit_path.empty()) return explicit_path;
  const auto match = parameter(nh, side + "_match");
  if (match.empty() && !fallback.empty()) return fallback;
  const auto directory = parameter(nh, "bag_dir");
  if (directory.empty() || match.empty()) throw std::runtime_error("Set " + side + "_bag or bag_dir and " + side + "_match");
  std::vector<std::string> matches;
  for (const auto& entry : fs::directory_iterator(directory))
    if (entry.is_regular_file() && entry.path().extension() == ".bag" &&
        entry.path().filename().string().find(match) != std::string::npos)
      matches.push_back(entry.path().string());
  if (matches.size() != 1) throw std::runtime_error("Expected exactly one " + side + " bag, found " + std::to_string(matches.size()));
  return matches.front();
}

static std_msgs::Header headerOf(const rosbag::MessageInstance& message) {
  if (auto image = message.instantiate<sensor_msgs::Image>()) return image->header;
  if (auto image = message.instantiate<sensor_msgs::CompressedImage>()) return image->header;
  throw std::runtime_error("Unsupported image message on " + message.getTopic());
}

static Index indexImages(rosbag::Bag& bag, const std::string& topic) {
  Index index;
  rosbag::View view(bag, rosbag::TopicQuery(topic));
  for (const auto& message : view) {
    auto header = headerOf(message);
    if (header.stamp.isZero() || header.frame_id.empty())
      throw std::runtime_error("Missing acquisition time or optical frame on " + topic);
    if (!index.emplace(header.stamp.toNSec(), message).second)
      throw std::runtime_error("Duplicate acquisition timestamp on " + topic);
  }
  if (index.empty()) throw std::runtime_error("No images on " + topic);
  return index;
}

static sensor_msgs::CameraInfo loadCalibration(rosbag::Bag& bag, const std::string& topic) {
  sensor_msgs::CameraInfo::ConstPtr selected;
  rosbag::View view(bag, rosbag::TopicQuery(topic));
  for (const auto& message : view) {
    auto info = message.instantiate<sensor_msgs::CameraInfo>();
    if (!info) throw std::runtime_error("Expected CameraInfo on " + topic);
    if (selected && (selected->K != info->K || selected->D != info->D || selected->R != info->R ||
                     selected->P != info->P || selected->width != info->width || selected->height != info->height ||
                     selected->header.frame_id != info->header.frame_id || selected->distortion_model != info->distortion_model))
      throw std::runtime_error("Calibration changes within the bag: " + topic);
    selected = info;
  }
  if (!selected) throw std::runtime_error("Missing calibration on " + topic);
  auto info = *selected;
  cv::Mat k, d, r, p;
  // Reuse the same CameraInfo conversion as the existing raw-bag viewer.
  cameraInfoToCalibration(info, k, d, r, p);
  if (!info.width || !info.height || info.header.frame_id.empty() ||
      !cv::checkRange(k) || !cv::checkRange(r) || !cv::checkRange(p) || (!d.empty() && !cv::checkRange(d)) ||
      k.at<double>(0, 0) <= 0 || k.at<double>(1, 1) <= 0 || p.at<double>(0, 0) <= 0 || p.at<double>(1, 1) <= 0 ||
      std::abs(cv::determinant(r) - 1.0) > 1e-5 || cv::norm(r.t()*r - cv::Mat::eye(3, 3, CV_64F)) > 1e-5)
    throw std::runtime_error("Invalid camera matrices on " + topic);
  if (info.binning_x > 1 || info.binning_y > 1 || info.roi.x_offset || info.roi.y_offset ||
      (info.roi.width && info.roi.width != info.width) || (info.roi.height && info.roi.height != info.height))
    throw std::runtime_error("Cropped/binned calibration requires explicit geometry handling: " + topic);
  return info;
}

static cv::Mat decodeImage(const rosbag::MessageInstance& message, bool color) {
  if (auto image = message.instantiate<sensor_msgs::Image>())
    return (color ? cv_bridge::toCvCopy(image, "bgr8") : cv_bridge::toCvCopy(image))->image;
  auto image = message.instantiate<sensor_msgs::CompressedImage>();
  if (!image) throw std::runtime_error("Expected image on " + message.getTopic());
  cv::Mat pixels;
  if (color) {
    if (!rectify_from_bag::turbojpeg_utils::decodeCompressedBGR(*image, pixels))
      throw std::runtime_error("Cannot decode color image on " + message.getTopic());
  } else {
    pixels = cv::imdecode(image->data, cv::IMREAD_UNCHANGED);
    // compressed_depth_image_transport prepends its 12-byte configuration
    // header. Only inspect pixel dimensions here; preserve the original payload.
    if (pixels.empty() && image->format.find("compressedDepth") != std::string::npos && image->data.size() > 12) {
      std::vector<uint8_t> payload(image->data.begin() + 12, image->data.end());
      pixels = cv::imdecode(payload, cv::IMREAD_UNCHANGED);
    }
  }
  if (pixels.empty()) throw std::runtime_error("Cannot decode image on " + message.getTopic());
  return pixels;
}

static void validateImage(const rosbag::MessageInstance& message, const cv::Mat& pixels,
                          const sensor_msgs::CameraInfo& info) {
  if (pixels.cols != static_cast<int>(info.width) || pixels.rows != static_cast<int>(info.height) ||
      headerOf(message).frame_id != info.header.frame_id)
    throw std::runtime_error("Image dimensions/optical frame disagree with CameraInfo on " + message.getTopic());
}

class CameraProcessor {
 public:
  CameraProcessor(sensor_msgs::CameraInfo info, bool already_rectified, int quality, std::string codec)
      : info_(std::move(info)), already_rectified_(already_rectified), quality_(quality), codec_(std::move(codec)) {
    cv::Mat k, d, r, p;
    cameraInfoToCalibration(info_, k, d, r, p);
    if (already_rectified_) {
      if (cv::norm(d) > 1e-10 || cv::norm(r - cv::Mat::eye(3, 3, CV_64F)) > 1e-10 ||
          cv::norm(k - p(cv::Rect(0, 0, 3, 3))) > 1e-5)
        throw std::runtime_error("input_rectified=true requires rectified CameraInfo (D=0, R=I, K=P[:3,:3])");
    } else if (info_.distortion_model == "equidistant" && info_.D.size() == 4) {
      cv::fisheye::initUndistortRectifyMap(k, d, r, p(cv::Rect(0, 0, 3, 3)),
                                         cv::Size(info_.width, info_.height), CV_32FC1, map_x_, map_y_);
    } else if ((info_.distortion_model == "plumb_bob" && info_.D.size() == 5) ||
               (info_.distortion_model == "rational_polynomial" && info_.D.size() == 8)) {
      cv::initUndistortRectifyMap(k, d, r, p(cv::Rect(0, 0, 3, 3)),
                                cv::Size(info_.width, info_.height), CV_32FC1, map_x_, map_y_);
    } else {
      throw std::runtime_error("Unsupported distortion model/coefficient count: " + info_.distortion_model);
    }
  }
  void write(const rosbag::MessageInstance& message, rosbag::Bag& output,
             const std::string& topic, const std::string& info_topic) {
    cv::Mat pixels = decodeImage(message, true);
    validateImage(message, pixels, info_);
    const auto header = headerOf(message);
    sensor_msgs::CompressedImage image;
    if (already_rectified_ && message.isType<sensor_msgs::CompressedImage>()) {
      image = *message.instantiate<sensor_msgs::CompressedImage>();
    } else {
      if (!already_rectified_) {
        cv::Mat rectified;
        cv::remap(pixels, rectified, map_x_, map_y_, cv::INTER_LINEAR);
        pixels = std::move(rectified);
      }
      image.header = header;
      image.format = codec_ == "png" ? "bgr8; png compressed bgr8" : "bgr8; jpeg compressed bgr8";
      // PNG avoids adding another lossy encoding step during a geometric repair.
      const bool encoded = codec_ == "png" ? cv::imencode(".png", pixels, image.data)
          : rectify_from_bag::turbojpeg_utils::encodeJpeg(pixels, image.data, quality_);
      if (!encoded)
        throw std::runtime_error("Cannot encode rectified image");
    }
    auto info = info_;
    info.header = header;
    // CameraInfo retains the calibrated K,D,R,P contract: P describes the
    // rectified image and includes the stereo baseline. Do not overwrite it
    // with a shared matrix or drop P's translation column.
    output.write(topic, header.stamp, image);
    output.write(info_topic, header.stamp, info);
  }
 private:
  sensor_msgs::CameraInfo info_;
  bool already_rectified_;
  int quality_;
  std::string codec_;
  cv::Mat map_x_, map_y_;
};

static void writePassthrough(const rosbag::MessageInstance& message, rosbag::Bag& output,
                            const sensor_msgs::CameraInfo& calibration,
                            const std::string& topic, const std::string& info_topic) {
  auto pixels = decodeImage(message, false);
  validateImage(message, pixels, calibration);
  auto info = calibration;
  info.header = headerOf(message);
  if (auto image = message.instantiate<sensor_msgs::Image>()) output.write(topic, info.header.stamp, *image);
  else output.write(topic, info.header.stamp, *message.instantiate<sensor_msgs::CompressedImage>());
  output.write(info_topic, info.header.stamp, info);
}

static int process(const ros::NodeHandle& nh) {
  bool already_rectified = true, allow_unmatched = false;
  int jpeg_quality = 95;
  nh.param("input_rectified", already_rectified, true);
  nh.param("allow_unmatched", allow_unmatched, false);
  nh.param("jpeg_quality", jpeg_quality, 95);
  if (jpeg_quality < 1 || jpeg_quality > 100) throw std::runtime_error("jpeg_quality must be in [1,100]");
  const auto codec = parameter(nh, "output_codec", "png");
  if (codec != "png" && codec != "jpeg") throw std::runtime_error("output_codec must be png or jpeg");
  const auto right_path = resolveBag(nh, "right");
  const auto left_path = resolveBag(nh, "left", right_path);
  const auto depth_path = resolveBag(nh, "depth");
  const auto confidence_path = resolveBag(nh, "confidence", depth_path);
  rosbag::Bag right_bag(right_path), left_bag(left_path), depth_bag(depth_path), confidence_bag(confidence_path);
  const auto right_topic = parameter(nh, "right_topic", "/boxi/zed2i/right/image_rect/compressed");
  const auto left_topic = parameter(nh, "left_topic", "/boxi/zed2i/left/image_rect/compressed");
  const auto depth_topic = parameter(nh, "depth_topic", "/boxi/zed2i/depth/image_raw/compressed");
  const auto confidence_topic = parameter(nh, "confidence_topic", "/boxi/zed2i/confidence/image_raw/compressed");
  auto right_info = loadCalibration(right_bag, parameter(nh, "right_info_topic", "/boxi/zed2i/right/camera_info"));
  auto left_info = loadCalibration(left_bag, parameter(nh, "left_info_topic", "/boxi/zed2i/left/camera_info"));
  auto depth_info = loadCalibration(depth_bag, parameter(nh, "depth_info_topic", "/boxi/zed2i/depth/camera_info"));
  const auto confidence_info_topic = parameter(nh, "confidence_info_topic");
  // ZED confidence is sampled on the depth pixel grid and has no separate
  // CameraInfo in the driver. Frame and dimensions are checked for every image.
  auto confidence_info = confidence_info_topic.empty() ? depth_info : loadCalibration(confidence_bag, confidence_info_topic);
  CameraProcessor right_processor(right_info, already_rectified, jpeg_quality, codec);
  CameraProcessor left_processor(left_info, already_rectified, jpeg_quality, codec);
  const auto right = indexImages(right_bag, right_topic);
  const auto left = indexImages(left_bag, left_topic);
  const auto depth = indexImages(depth_bag, depth_topic);
  const auto confidence = indexImages(confidence_bag, confidence_topic);
  box_export::AtomicDirectory dataset(parameter(nh, "output_dir"));
  rosbag::Bag images((dataset.path()/"images.bag").string(), rosbag::bagmode::Write);
  rosbag::Bag depths((dataset.path()/"depth.bag").string(), rosbag::bagmode::Write);
  images.setCompression(rosbag::compression::LZ4);
  depths.setCompression(rosbag::compression::LZ4);
  size_t written = 0, unmatched = 0;
  for (const auto& entry : right) {
    const auto l = left.find(entry.first), d = depth.find(entry.first), c = confidence.find(entry.first);
    if (l == left.end() || d == depth.end() || c == confidence.end()) {
      ++unmatched;
      if (!allow_unmatched) throw std::runtime_error("Unmatched stereo/depth timestamp; select allow_unmatched to export the explicit intersection");
      continue;
    }
    right_processor.write(entry.second, images, parameter(nh, "out_right_topic", "/boxi/zed2i/right/image_rect/compressed"),
                          parameter(nh, "out_right_info_topic", "/boxi/zed2i/right/camera_info_rect"));
    left_processor.write(l->second, images, parameter(nh, "out_left_topic", "/boxi/zed2i/left/image_rect/compressed"),
                         parameter(nh, "out_left_info_topic", "/boxi/zed2i/left/camera_info_rect"));
    writePassthrough(d->second, depths, depth_info, parameter(nh, "out_depth_topic", depth_topic),
                     parameter(nh, "out_depth_info_topic", "/boxi/zed2i/depth/camera_info"));
    writePassthrough(c->second, depths, confidence_info, parameter(nh, "out_confidence_topic", confidence_topic),
                     parameter(nh, "out_confidence_info_topic", "/boxi/zed2i/confidence/camera_info"));
    ++written;
  }
  if (!written) throw std::runtime_error("No synchronized frames; no dataset published");
  if (!allow_unmatched && (written != left.size() || written != depth.size() || written != confidence.size()))
    throw std::runtime_error("Additional unmatched messages in a selected stream");
  images.close();
  depths.close();
  std::ofstream summary(dataset.path()/"processing_summary.json");
  summary.exceptions(std::ios::failbit | std::ios::badbit);
  summary << "{\"frames\":" << written << ",\"input_rectified\":" << (already_rectified ? "true" : "false")
          << ",\"right_unmatched\":" << unmatched << ",\"left_unmatched\":" << left.size()-written
          << ",\"depth_unmatched\":" << depth.size()-written << ",\"confidence_unmatched\":" << confidence.size()-written << "}\n";
  summary.close();
  dataset.commit();
  std::cout << "Published " << written << " synchronized frames\n";
  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rectify_and_write_bag");
  try {
    return process(ros::NodeHandle("~"));
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("Dataset processing failed: " << error.what());
    return 1;
  }
}

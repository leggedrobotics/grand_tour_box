#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <zed2i_recording_driver_msgs/StartRecordingSVO.h>

#include <atomic>
#include <memory>
#include <thread>
#include <zed2i_recording_driver/recording_session.hpp>
#include <zed2i_recording_driver/sensor_messages.hpp>

namespace {
class ZedBackend : public zed_recording::RecordingBackend {
 public:
  explicit ZedBackend(sl::Camera& camera) : camera_(camera) {}
  sl::SVO_COMPRESSION_MODE compression = sl::SVO_COMPRESSION_MODE::H264;
  std::string enable(const std::string& path) override {
    sl::RecordingParameters params;
    params.video_filename = sl::String(path.c_str());
    params.compression_mode = compression;
    const auto error = camera_.enableRecording(params);
    return error == sl::ERROR_CODE::SUCCESS ? "" : sl::toString(error).c_str();
  }
  zed_recording::Frame grab() override {
    const auto error = camera_.grab();
    if (error > sl::ERROR_CODE::SUCCESS) return {0, sl::toString(error).c_str()};
    // Negative SDK codes are warnings with an available image (for example low
    // image quality). Preserve raw capture, as main did, and expose the warning.
    if (error != sl::ERROR_CODE::SUCCESS) ROS_WARN_STREAM_THROTTLE(5.0, "ZED capture warning: " << sl::toString(error));
    const auto status = camera_.getRecordingStatus();
    if (!status.is_recording || status.is_paused) return {0, "SDK recording stopped or paused unexpectedly"};
    // status can initially be false because the encoder buffers its first frame.
    // The session permits bounded startup latency, then requires successful writes.
    return {camera_.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds(), "", status.status};
  }
  void disable() override { camera_.disableRecording(); }

 private:
  sl::Camera& camera_;
};

sl::RESOLUTION resolutionFor(const std::string& value, int fps) {
  if (value == "VGA" && (fps == 15 || fps == 30 || fps == 60 || fps == 100)) return sl::RESOLUTION::VGA;
  if (value == "HD720" && (fps == 15 || fps == 30 || fps == 60)) return sl::RESOLUTION::HD720;
  if (value == "HD1080" && (fps == 15 || fps == 30)) return sl::RESOLUTION::HD1080;
  if (value == "HD2K" && fps == 15) return sl::RESOLUTION::HD2K;
  throw std::runtime_error("Invalid camera_resolution/camera_fps combination");
}

class ZedRecordingNode {
 public:
  ZedRecordingNode() : nh_("~"), backend_(camera_) {
    std::string resolution, compression;
    int fps, camera_id, minimum_mb;
    nh_.param<std::string>("camera_resolution", resolution, "HD1080");
    nh_.param<int>("camera_fps", fps, 30);
    nh_.param<int>("camera_id", camera_id, 0);
    nh_.param<int>("min_free_space_mb", minimum_mb, 1024);
    nh_.param<std::string>("compression", compression, "H264");
    nh_.param<std::string>("imu_frame_id", imu_frame_, "zed2i_imu_link");
    nh_.param<std::string>("mag_frame_id", mag_frame_, "zed2i_mag_link");
    // Pressure is scalar; use the existing camera center rather than inventing a
    // barometer extrinsic that the SDK does not supply.
    nh_.param<std::string>("baro_frame_id", baro_frame_, "zed2i_camera_center");
    nh_.param<bool>("publish_magnetometer", publish_mag_, true);
    nh_.param<bool>("publish_barometer", publish_baro_, true);
    if (minimum_mb < 0 || camera_id < 0) throw std::runtime_error("Negative space/camera parameter");
    if (imu_frame_.empty() || mag_frame_.empty() || baro_frame_.empty() || imu_frame_ == mag_frame_)
      throw std::runtime_error("Sensor frames must be named; IMU and magnetometer frames must differ");
    if (compression == "H265")
      backend_.compression = sl::SVO_COMPRESSION_MODE::H265;
    else if (compression != "H264")
      throw std::runtime_error("compression must be H264 or H265");

    sl::InitParameters params;
    params.camera_resolution = resolutionFor(resolution, fps);
    params.camera_fps = fps;
    params.input.setFromCameraID(camera_id);
    params.depth_mode = sl::DEPTH_MODE::NONE;
    // Match the wrapper pinned on main: ROS body axes, not SDK IMAGE axes.
    params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    params.coordinate_units = sl::UNIT::METER;
    const auto error = camera_.open(params);
    if (error != sl::ERROR_CODE::SUCCESS)
      throw std::runtime_error(std::string("Cannot open ZED: ") + sl::toString(error).c_str());
    const auto info = camera_.getCameraInformation();
    if (info.camera_model != sl::MODEL::ZED2i) {
      camera_.close();
      throw std::runtime_error("This recorder's frame contract requires a ZED2i");
    }
    session_.reset(new zed_recording::RecordingSession(backend_, uintmax_t(minimum_mb) * 1024 * 1024));
    hz_pub_ = nh_.advertise<std_msgs::Float32>("recording_hz", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1000);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("mag", 200);
    baro_pub_ = nh_.advertise<sensor_msgs::FluidPressure>("baro", 200);
    if (publish_mag_ && info.sensors_configuration.magnetometer_parameters.isAvailable) {
      // SDK supplies magnetometer -> IMU. Keep main's calibrated camera -> IMU
      // TF; publish only this new child, with factory extrinsics in ROS axes/meters.
      auto transform = info.sensors_configuration.imu_magnetometer_transform;
      const auto translation = transform.getTranslation();
      const auto rotation = transform.getOrientation();
      geometry_msgs::TransformStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = imu_frame_;
      msg.child_frame_id = mag_frame_;
      msg.transform.translation.x = translation.x;
      msg.transform.translation.y = translation.y;
      msg.transform.translation.z = translation.z;
      msg.transform.rotation.x = rotation.x;
      msg.transform.rotation.y = rotation.y;
      msg.transform.rotation.z = rotation.z;
      msg.transform.rotation.w = rotation.w;
      tf_pub_.sendTransform(msg);
    }
    // Advertise only after camera initialization and state construction have succeeded.
    recording_srv_ = nh_.advertiseService("start_recording_svo", &ZedRecordingNode::recording, this);
    status_srv_ = nh_.advertiseService("get_recording_status", &ZedRecordingNode::status, this);
    status_timer_ = nh_.createWallTimer(ros::WallDuration(1.0), &ZedRecordingNode::publishStatus, this);
    ROS_INFO_STREAM("ZED2i recorder ready: " << resolution << " at " << fps << " Hz, " << compression);
  }

  ~ZedRecordingNode() {
    running_ = false;
    if (grab_thread_.joinable()) grab_thread_.join();
    if (sensor_thread_.joinable()) sensor_thread_.join();
    if (session_) session_->stop();
    camera_.close();
  }

  void spin() {
    running_ = true;
    grab_thread_ = std::thread([this] {
      while (running_ && ros::ok()) {
        session_->tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    sensor_thread_ = std::thread(&ZedRecordingNode::sensors, this);
    ros::spin();
  }

 private:
  bool recording(zed2i_recording_driver_msgs::StartRecordingSVO::Request& req,
                 zed2i_recording_driver_msgs::StartRecordingSVO::Response& res) {
    if (req.start_recording) {
      res.success = session_->start(req.video_filename);
      if (!res.success) ROS_ERROR_STREAM("SVO start rejected: " << session_->status().error);
    } else {
      // A stop must also report an earlier write failure, even if the client has
      // not polled the status service since that failure occurred.
      res.success = session_->stop();
    }
    return true;
  }
  bool status(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    const auto current = session_->status();
    res.success = current.active;
    res.message = (current.active ? "Recording: " : "Stopped: ") + current.filename;
    if (!current.error.empty()) res.message += " | " + current.error;
    return true;
  }
  void publishStatus(const ros::WallTimerEvent&) {
    const auto current = session_->status();
    std_msgs::Float32 hz;
    hz.data = current.hz;
    hz_pub_.publish(hz);
    if (!current.error.empty()) ROS_ERROR_STREAM_THROTTLE(5.0, current.error);
  }
  void sensors() {
    uint64_t last_imu = 0, last_mag = 0, last_baro = 0;
    sl::SensorsData data;
    while (running_ && ros::ok()) {
      // Supported SDK path: CURRENT sensor polling independent of blocking grab().
      // Each sensor has its own timestamp/rate; mag/baro are not gated on new IMU.
      if (camera_.getSensorsData(data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) {
        if (data.imu.is_available && data.imu.timestamp.getNanoseconds() > last_imu &&
            zed_recording::finite(data.imu.angular_velocity) && zed_recording::finite(data.imu.linear_acceleration)) {
          imu_pub_.publish(zed_recording::imuMessage(data.imu, imu_frame_));
          last_imu = data.imu.timestamp.getNanoseconds();
        }
        if (publish_mag_ && data.magnetometer.is_available && data.magnetometer.timestamp.getNanoseconds() > last_mag &&
            zed_recording::finite(data.magnetometer.magnetic_field_calibrated)) {
          mag_pub_.publish(zed_recording::magneticMessage(data.magnetometer, mag_frame_));
          last_mag = data.magnetometer.timestamp.getNanoseconds();
        }
        if (publish_baro_ && data.barometer.is_available && data.barometer.timestamp.getNanoseconds() > last_baro &&
            std::isfinite(data.barometer.pressure) && data.barometer.pressure > 0) {
          baro_pub_.publish(zed_recording::pressureMessage(data.barometer, baro_frame_));
          last_baro = data.barometer.timestamp.getNanoseconds();
        }
      } else {
        ROS_WARN_THROTTLE(5.0, "ZED sensor read failed");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  ros::NodeHandle nh_;
  sl::Camera camera_;
  ZedBackend backend_;
  std::unique_ptr<zed_recording::RecordingSession> session_;
  ros::Publisher hz_pub_, imu_pub_, mag_pub_, baro_pub_;
  ros::ServiceServer recording_srv_, status_srv_;
  ros::WallTimer status_timer_;
  tf2_ros::StaticTransformBroadcaster tf_pub_;
  std::string imu_frame_, mag_frame_, baro_frame_;
  bool publish_mag_ = true, publish_baro_ = true;
  std::atomic<bool> running_{false};
  std::thread grab_thread_, sensor_thread_;
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "zed2i_recording_driver");
  try {
    ZedRecordingNode node;
    node.spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
  return 0;
}

#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <cmath>

// Use <filesystem> if available, otherwise fall back to <experimental/filesystem>
#if __has_include(<filesystem>)
#  include <filesystem>
   namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#  include <experimental/filesystem>
   namespace fs = std::experimental::filesystem;
#else
#  error "No filesystem support available"
#endif

#include <sl/Camera.hpp>
#include <zed2i_recording_driver_msgs/StartRecordingSVO.h>

class ZedRecordingNode {
private:
    ros::NodeHandle nh_;

    // Publishers
    ros::Publisher hz_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Publisher baro_pub_;

    // Services
    ros::ServiceServer start_stop_srv_;
    ros::ServiceServer status_srv_;

    // ZED camera
    sl::Camera zed_;

    // Recording thread
    std::atomic<bool> recording_{false};
    std::thread record_thread_;

    // Synchronization
    std::mutex mutex_;        // protects video_filename_, recording_, timestamps_
    std::mutex zed_ctrl_mtx_; // serializes enableRecording() / disableRecording()

    // Sensors thread
    std::atomic<bool> sensors_running_{false};
    std::thread sensors_thread_;

    // Paths / status
    std::string video_filename_;
    std::string last_error_;

    // Recording Hz window
    std::deque<sl::Timestamp> timestamps_;

    // Params
    std::string param_resolution_{"HD1080"};
    int param_fps_{30};
    uint64_t param_min_free_space_mb_{1024}; // 1 GiB default
    int tmp_mb = static_cast<int>(param_min_free_space_mb_);
    std::string imu_frame_id_{"zed2i_imu_link"};
    std::string mag_frame_id_{"zed2i_mag_link"};
    std::string baro_frame_id_{"zed2i_baro_link"};
    bool publish_magnetometer_{true};
    bool publish_barometer_{true};
    int imu_queue_size_{1000};
    int mag_queue_size_{200};
    int baro_queue_size_{200};

public:
    ZedRecordingNode() : nh_("~") {
        // Params
        nh_.param<std::string>("camera_resolution", param_resolution_, param_resolution_);
        nh_.param<int>("camera_fps", param_fps_, param_fps_);
        // Read free-space parameter as int to avoid type mismatch, then cast back
        nh_.param<int>("min_free_space_mb", tmp_mb, tmp_mb);
        param_min_free_space_mb_ = static_cast<uint64_t>(tmp_mb);
        nh_.param<std::string>("imu_frame_id", imu_frame_id_, imu_frame_id_);
        nh_.param<std::string>("mag_frame_id", mag_frame_id_, mag_frame_id_);
        nh_.param<std::string>("baro_frame_id", baro_frame_id_, baro_frame_id_);
        nh_.param<bool>("publish_magnetometer", publish_magnetometer_, publish_magnetometer_);
        nh_.param<bool>("publish_barometer", publish_barometer_, publish_barometer_);
        nh_.param<int>("imu_queue_size", imu_queue_size_, imu_queue_size_);
        nh_.param<int>("mag_queue_size", mag_queue_size_, mag_queue_size_);
        nh_.param<int>("baro_queue_size", baro_queue_size_, baro_queue_size_);

        // Publishers
        hz_pub_   = nh_.advertise<std_msgs::Float32>("recording_hz", 10);
        imu_pub_  = nh_.advertise<sensor_msgs::Imu>("imu", imu_queue_size_);
        mag_pub_  = nh_.advertise<sensor_msgs::MagneticField>("mag", mag_queue_size_);
        baro_pub_ = nh_.advertise<sensor_msgs::FluidPressure>("baro", baro_queue_size_);

        // Services
        start_stop_srv_ = nh_.advertiseService(
            "start_recording_svo", &ZedRecordingNode::recordingService, this);
        status_srv_ = nh_.advertiseService(
            "get_recording_status", &ZedRecordingNode::getStatusService, this);

        ROS_INFO("Initializing ZedRecordingNode...");
        std::cout << "WARNING: This node is designed for ZED2i cameras only. Ensure compatibility before proceeding."
                  << std::endl;

        openCamera();
    }

    ~ZedRecordingNode() {
        stopSensorsThreadIfRunning();
        stopRecordingThreadIfRunning(); // also disables recording if active
        zed_.close();
    }

private:
    static sl::RESOLUTION getResolutionFromString(const std::string &resolution) {
        if (resolution == "VGA")   return sl::RESOLUTION::VGA;
        if (resolution == "HD720") return sl::RESOLUTION::HD720;
        if (resolution == "HD1080")return sl::RESOLUTION::HD1080;
        if (resolution == "HD2K")  return sl::RESOLUTION::HD2K;
        ROS_WARN("Unknown resolution '%s'. Defaulting to HD1080.", resolution.c_str());
        return sl::RESOLUTION::HD1080;
    }

    static bool validateFramerateForResolution(const std::string& resolution, int fps) {
        if (resolution == "VGA")    return (fps == 15 || fps == 30 || fps == 60 || fps == 100);
        if (resolution == "HD720")  return (fps == 15 || fps == 30 || fps == 60);
        if (resolution == "HD1080") return (fps == 15 || fps == 30);
        if (resolution == "HD2K")   return (fps == 15);
        ROS_WARN("Unknown resolution '%s'. Defaulting to HD1080.", resolution.c_str());
        return (fps == 15 || fps == 30);
    }

    static ros::Time toRosTime(const sl::Timestamp &ts) {
        ros::Time t;
        t.fromNSec(static_cast<uint64_t>(ts.getNanoseconds()));
        return t;
    }

    void openCamera() {
        if (!validateFramerateForResolution(param_resolution_, param_fps_)) {
            ROS_FATAL("Invalid framerate %d for resolution %s. Shutting down.",
                      param_fps_, param_resolution_.c_str());
            ros::shutdown();
            return;
        }

        sl::InitParameters init_params;
        init_params.camera_resolution = getResolutionFromString(param_resolution_);
        init_params.depth_mode        = sl::DEPTH_MODE::NONE;
        init_params.camera_fps        = param_fps_;
        init_params.input.setFromCameraID(0);

        sl::ERROR_CODE err = zed_.open(init_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            ROS_FATAL_STREAM("ZED camera cannot be opened, error: " << sl::toString(err));
            ros::shutdown();
            return;
        }

        const auto camera_info = zed_.getCameraInformation();
        const auto camera_conf = camera_info.camera_configuration;

        std::cout << "\nZED Model                 : " << sl::toString(camera_info.camera_model) << std::endl;
        std::cout << "ZED Serial Number         : " << camera_info.serial_number << std::endl;
        std::cout << "ZED Camera Firmware       : " << camera_conf.firmware_version
                  << "/" << camera_info.sensors_configuration.firmware_version << std::endl;
        std::cout << "ZED Camera Resolution     : " << camera_conf.resolution.width
                  << "x" << camera_conf.resolution.height << std::endl;
        std::cout << "ZED Camera FPS            : " << zed_.getInitParameters().camera_fps << std::endl;

        ROS_INFO_STREAM("ZED camera opened. Resolution "
                        << camera_conf.resolution.width << "x" << camera_conf.resolution.height
                        << " @ " << init_params.camera_fps << " FPS");

        // Start sensors thread
        sensors_running_ = true;
        sensors_thread_  = std::thread(&ZedRecordingNode::sensorsLoop, this);
    }

    // --- Services ---

    // Start/stop service with definitive ACK/NACK
    bool recordingService(zed2i_recording_driver_msgs::StartRecordingSVO::Request &req,
                          zed2i_recording_driver_msgs::StartRecordingSVO::Response &res) {
        // Acquire state lock for start/stop decisions
        std::unique_lock<std::mutex> state_lock(mutex_);

        if (req.start_recording) {
            if (recording_) {
                ROS_INFO_STREAM("Start requested but already recording. File: " << video_filename_);
                res.success = true; // idempotent
                return true;
            }

            // Preflight
            std::string resolved;
            std::string err;
            if (!preflightOutputPath(req.video_filename, resolved, err)) {
                last_error_ = err;
                ROS_ERROR_STREAM("Preflight failed: " << err);
                res.success = false;
                return true;
            }

            // Enable recording (serialize via zed_ctrl_mtx_)
            {
                std::lock_guard<std::mutex> ctrl_lock(zed_ctrl_mtx_);
                sl::RecordingParameters rec_params;
                rec_params.compression_mode = sl::SVO_COMPRESSION_MODE::H265;
                rec_params.video_filename   = sl::String(resolved.c_str());

                sl::ERROR_CODE rc = zed_.enableRecording(rec_params);
                if (rc != sl::ERROR_CODE::SUCCESS) {
                    last_error_ = std::string("enableRecording failed: ") + sl::toString(rc).c_str();
                    ROS_ERROR_STREAM(last_error_);
                    res.success = false;
                    return true;
                }
            }

            // Start recording thread
            video_filename_ = resolved;
            last_error_.clear();
            timestamps_.clear();
            recording_ = true;

            record_thread_ = std::thread(&ZedRecordingNode::grabLoop, this);

            ROS_INFO_STREAM("Recording started: " << video_filename_);
            res.success = true;
            return true;
        } else {
            // Stop requested
            if (!recording_) {
                ROS_INFO("Stop requested but already stopped.");
                res.success = true;
                return true;
            }

            // Signal and extract thread handle
            recording_ = false;
            std::thread local;
            if (record_thread_.joinable()) {
                local.swap(record_thread_);
            }
            // Release state lock before join
            state_lock.unlock();

            if (local.joinable()) {
                local.join();
            }

            // Disable recording under serialization lock
            {
                std::lock_guard<std::mutex> ctrl_lock(zed_ctrl_mtx_);
                zed_.disableRecording();
            }

            // Clean up error state under lock
            {
                std::lock_guard<std::mutex> lock(mutex_);
                last_error_.clear();
            }

            ROS_INFO_STREAM("Recording stopped: " << video_filename_);
            res.success = true;
            return true;
        }
    }

    // Status service: returns whether recording is active plus last error and filename
    bool getStatusService(std_srvs::Trigger::Request &,
                          std_srvs::Trigger::Response &resp) {
        std::lock_guard<std::mutex> lock(mutex_);
        resp.success = recording_.load();
        std::ostringstream oss;
        oss << (recording_ ? "Recording" : "Idle");
        if (!video_filename_.empty()) {
            oss << " | file=" << video_filename_;
        }
        if (!last_error_.empty()) {
            oss << " | last_error=" << last_error_;
        }
        resp.message = oss.str();
        return true;
    }

    // --- Helpers ---

    // Stop recording thread if active and disable recording
    void stopRecordingThreadIfRunning() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (recording_) recording_ = false;
        }
        std::thread local;
        if (record_thread_.joinable()) {
            local.swap(record_thread_);
        }
        if (local.joinable()) {
            local.join();
        }
        // Disable recording even if not enabled; safe
        {
            std::lock_guard<std::mutex> ctrl_lock(zed_ctrl_mtx_);
            zed_.disableRecording();
        }
    }

    // Stop sensors thread if running
    void stopSensorsThreadIfRunning() {
        if (sensors_running_) {
            sensors_running_ = false;
            if (sensors_thread_.joinable()) sensors_thread_.join();
        }
    }

    // Preflight check for output path
    bool preflightOutputPath(const std::string& input_filename,
                             std::string& resolved_out,
                             std::string& err_out) const {
        try {
            fs::path p(input_filename.empty() ? "output.svo" : input_filename);
            if (!p.is_absolute()) p = fs::absolute(p);

            fs::path dir = p.parent_path();
            if (dir.empty()) dir = fs::current_path();

            if (!fs::exists(dir) || !fs::is_directory(dir)) {
                err_out = "Output directory does not exist: " + dir.string();
                return false;
            }

            const auto sp = fs::space(dir);
            const uint64_t required_bytes = param_min_free_space_mb_ * 1024ULL * 1024ULL;
            if (sp.available < required_bytes) {
                err_out = "Insufficient free space in " + dir.string() + " (available "
                          + std::to_string(sp.available / (1024ULL * 1024ULL)) + " MB, required >= "
                          + std::to_string(param_min_free_space_mb_) + " MB)";
                return false;
            }

            fs::path testfile = dir / ".zed_svo_write_test.tmp";
            {
                std::ofstream ofs(testfile.string(), std::ios::binary);
                if (!ofs.good()) {
                    err_out = "Directory is not writable: " + dir.string();
                    return false;
                }
            }
            (void)fs::remove(testfile);

            resolved_out = p.string();
            return true;
        } catch (const std::exception &e) {
            err_out = std::string("Path preflight exception: ") + e.what();
            return false;
        }
    }

    // Grab loop publishes ~recording_hz
    void grabLoop() {
        timestamps_.clear();

        while (ros::ok() && recording_.load(std::memory_order_relaxed)) {
            const sl::ERROR_CODE grc = zed_.grab();
            if (grc == sl::ERROR_CODE::SUCCESS) {
                const auto ts = zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE);
                timestamps_.push_back(ts);

                if (timestamps_.size() > 10) {
                    timestamps_.pop_front();

                    const int window_size = static_cast<int>(timestamps_.size());
                    const long long span_ns =
                        timestamps_.back().getNanoseconds()
                        - timestamps_.front().getNanoseconds();

                    if (window_size > 1 && span_ns > 0) {
                        const double avg_dt = (static_cast<double>(span_ns) / 1e9)
                                              / (window_size - 1);
                        const double avg_hz = 1.0 / avg_dt;

                        std_msgs::Float32 msg;
                        msg.data = static_cast<float>(avg_hz);
                        hz_pub_.publish(msg);

                        ROS_INFO_STREAM_THROTTLE(10.0,
                                                 "Publishing recording frequency: "
                                                     << msg.data << " Hz");
                    }
                }
            } else {
                // include .c_str() to match printf-style formatting
                ROS_WARN_THROTTLE(5.0,
                                  "zed_.grab() returned %s",
                                  sl::toString(grc).c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
    }

    // Sensors loop (non-blocking, independent of recording)
    void sensorsLoop() {
        // Helper for tracking new sensor data
        struct TimestampHandler {
            sl::Timestamp ts_imu = 0, ts_baro = 0, ts_mag = 0;
            inline bool isNew(sl::Timestamp& curr, sl::Timestamp& ref) {
                bool newer = curr > ref;
                if (newer) ref = curr;
                return newer;
            }
            inline bool isNew(sl::SensorsData::IMUData& imu) {
                return isNew(imu.timestamp, ts_imu);
            }
            inline bool isNew(sl::SensorsData::MagnetometerData& mag) {
                return isNew(mag.timestamp, ts_mag);
            }
            inline bool isNew(sl::SensorsData::BarometerData& baro) {
                return isNew(baro.timestamp, ts_baro);
            }
        } ts_guard;

        sl::SensorsData sensors_data;

        // Constants
        constexpr double DEG2RAD = M_PI / 180.0;
        constexpr double UT2T    = 1e-6;  // microtesla -> tesla
        constexpr double HPA2PA  = 100.0; // hPa -> Pa

        while (ros::ok() && sensors_running_.load(std::memory_order_relaxed)) {
            const sl::ERROR_CODE rc =
                zed_.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT);
            if (rc == sl::ERROR_CODE::SUCCESS) {
                // IMU (highest rate)
                if (ts_guard.isNew(sensors_data.imu)) {
                    sensor_msgs::Imu imu_msg;
                    imu_msg.header.stamp    = toRosTime(sensors_data.imu.timestamp);
                    imu_msg.header.frame_id = imu_frame_id_;
                    imu_msg.orientation_covariance[0] = -1.0; // orientation not published

                    // Angular velocity (deg/s -> rad/s)
                    const sl::float3& av = sensors_data.imu.angular_velocity;
                    imu_msg.angular_velocity.x = av.x * DEG2RAD;
                    imu_msg.angular_velocity.y = av.y * DEG2RAD;
                    imu_msg.angular_velocity.z = av.z * DEG2RAD;
                    imu_msg.angular_velocity_covariance[0] = -1.0;

                    // Linear acceleration (already m/s^2)
                    const sl::float3& la = sensors_data.imu.linear_acceleration;
                    imu_msg.linear_acceleration.x = la.x;
                    imu_msg.linear_acceleration.y = la.y;
                    imu_msg.linear_acceleration.z = la.z;
                    imu_msg.linear_acceleration_covariance[0] = -1.0;

                    imu_pub_.publish(imu_msg);

                    // Magnetometer (if updated and enabled)
                    if (publish_magnetometer_ && ts_guard.isNew(sensors_data.magnetometer)) {
                        sensor_msgs::MagneticField mag_msg;
                        mag_msg.header.stamp    = toRosTime(sensors_data.magnetometer.timestamp);
                        mag_msg.header.frame_id = mag_frame_id_;
                        const sl::float3& mf = sensors_data.magnetometer.magnetic_field_calibrated;
                        mag_msg.magnetic_field.x = mf.x * UT2T;
                        mag_msg.magnetic_field.y = mf.y * UT2T;
                        mag_msg.magnetic_field.z = mf.z * UT2T;
                        // covariance left default (unknown)
                        mag_pub_.publish(mag_msg);
                    }

                    // Barometer (if updated and enabled)
                    if (publish_barometer_ && ts_guard.isNew(sensors_data.barometer)) {
                        sensor_msgs::FluidPressure baro_msg;
                        baro_msg.header.stamp    = toRosTime(sensors_data.barometer.timestamp);
                        baro_msg.header.frame_id = baro_frame_id_;
                        baro_msg.fluid_pressure  = sensors_data.barometer.pressure * HPA2PA;
                        // variance unknown
                        baro_pub_.publish(baro_msg);
                    }
                }
            } else {
                // use c_str() with format specifier
                ROS_WARN_THROTTLE(5.0,
                                  "getSensorsData() returned %s",
                                  sl::toString(rc).c_str());
            }

            // Iterate fast to avoid missing samples; short sleep to reduce CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "zed2i_recording_driver");

    // Use an AsyncSpinner with at least 2 threads for responsiveness
    ros::NodeHandle pnh("~");
    int spinner_threads = 2;
    pnh.param<int>("spinner_threads", spinner_threads, spinner_threads);
    if (spinner_threads < 2) spinner_threads = 2;
    ros::AsyncSpinner spinner(spinner_threads);
    spinner.start();

    {
        ZedRecordingNode node;
        ros::waitForShutdown(); // keep process alive; default SIGINT triggers cleanup
    }

    return 0;
}

#include "rerun_camera_imu_viz.h"
#include <algorithm>
#include <cmath>

const std::vector<rerun::Color> RerunCameraImuViz::kCameraColors = {
        {138, 63,  252},  // purple
        {51,  177, 255},  // cyan
        {250, 77,  86},   // red
        {111, 220, 140},  // green
        {209, 39,  113},  // magenta
        {160, 50,  200},  // violet
        {200, 100, 150},  // pink
};

RerunCameraImuViz::RerunCameraImuViz(rerun::RecordingStream& rec) : rec_(rec) {}

rerun::Color RerunCameraImuViz::colorForCamera(const std::string& camera_name) {
    if (camera_color_index_.find(camera_name) == camera_color_index_.end())
        camera_color_index_[camera_name] = camera_color_index_.size();
    return kCameraColors[camera_color_index_.at(camera_name) % kCameraColors.size()];
}

void RerunCameraImuViz::vizDetections(const std::string& camera_name,
                                      double timestamp_s,
                                      const std::vector<std::array<float, 2>>& corners_2d) {
    if (corners_2d.empty()) return;
    rec_.set_time_timestamp_secs_since_epoch(kTimeline, timestamp_s);
    rec_.log(std::string(kBase) + camera_name + "/detections",
             rerun::Points2D(corners_2d).with_colors(colorForCamera(camera_name)).with_radii(2.f));
}

void RerunCameraImuViz::vizImage(const std::string& camera_name,
                                 double timestamp_s,
                                 const std::vector<uint8_t>& image_data,
                                 uint32_t width,
                                 uint32_t height) {
    if (image_data.empty()) return;
    rec_.set_time_timestamp_secs_since_epoch(kTimeline, timestamp_s);
    rec_.log(std::string(kBase) + camera_name + "/image",
             rerun::Image::from_grayscale8(image_data, {width, height}));
}


void RerunCameraImuViz::vizCameraPose(const std::string& camera_name,
                                      double timestamp_s,
                                      const Eigen::Affine3d& T_camera_board) {
    const auto color = colorForCamera(camera_name);
    const std::string pos_base  = std::string(kBase) + camera_name + "/position";
    const std::string ori_base  = std::string(kBase) + camera_name + "/orientation";

    // Log series style once (static — safe to repeat, rerun deduplicates).
    for (const auto& path : {pos_base + "/x", pos_base + "/y", pos_base + "/z",
                              ori_base + "/roll", ori_base + "/pitch", ori_base + "/yaw"}) {
        rec_.log_static(path, rerun::SeriesLines().with_colors(color).with_widths(2));
    }

    rec_.set_time_timestamp_secs_since_epoch(kTimeline, timestamp_s);

    const Eigen::Vector3d t = T_camera_board.translation();
    rec_.log(pos_base + "/x", rerun::Scalars(t.x()));
    rec_.log(pos_base + "/y", rerun::Scalars(t.y()));
    rec_.log(pos_base + "/z", rerun::Scalars(t.z()));

    // ZYX Euler angles → roll (X), pitch (Y), yaw (Z).
    const Eigen::Vector3d rpy = T_camera_board.rotation().eulerAngles(2, 1, 0).reverse();
    rec_.log(ori_base + "/roll",  rerun::Scalars(rpy.x()));
    rec_.log(ori_base + "/pitch", rerun::Scalars(rpy.y()));
    rec_.log(ori_base + "/yaw",   rerun::Scalars(rpy.z()));
}

void RerunCameraImuViz::vizImuState(double timestamp_s,
                                    const Eigen::Vector3d& position,
                                    const Eigen::Vector3d& velocity,
                                    const Eigen::Quaterniond& orientation,
                                    const Eigen::Vector3d& acc_bias,
                                    const Eigen::Vector3d& gyro_bias) {
    static constexpr char kImu[] = "world/imu/";
    static const rerun::Color kColor{241, 194, 27};  // yellow — single IMU source

    // Register series styles once.
    static bool styled = false;
    if (!styled) {
        styled = true;
        for (const char* path : {
                "world/imu/position/x",    "world/imu/position/y",    "world/imu/position/z",
                "world/imu/velocity/x",    "world/imu/velocity/y",    "world/imu/velocity/z",
                "world/imu/orientation/roll", "world/imu/orientation/pitch", "world/imu/orientation/yaw",
                "world/imu/acc_bias/x",    "world/imu/acc_bias/y",    "world/imu/acc_bias/z",
                "world/imu/gyro_bias/x",   "world/imu/gyro_bias/y",   "world/imu/gyro_bias/z"}) {
            rec_.log_static(path, rerun::SeriesLines().with_colors(kColor).with_widths(2));
        }
    }

    rec_.set_time_timestamp_secs_since_epoch(kTimeline, timestamp_s);

    rec_.log(std::string(kImu) + "position/x", rerun::Scalars(position.x()));
    rec_.log(std::string(kImu) + "position/y", rerun::Scalars(position.y()));
    rec_.log(std::string(kImu) + "position/z", rerun::Scalars(position.z()));

    rec_.log(std::string(kImu) + "velocity/x", rerun::Scalars(velocity.x()));
    rec_.log(std::string(kImu) + "velocity/y", rerun::Scalars(velocity.y()));
    rec_.log(std::string(kImu) + "velocity/z", rerun::Scalars(velocity.z()));

    const Eigen::Vector3d rpy = orientation.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
    rec_.log(std::string(kImu) + "orientation/roll",  rerun::Scalars(rpy.x()));
    rec_.log(std::string(kImu) + "orientation/pitch", rerun::Scalars(rpy.y()));
    rec_.log(std::string(kImu) + "orientation/yaw",   rerun::Scalars(rpy.z()));

    rec_.log(std::string(kImu) + "acc_bias/x",  rerun::Scalars(acc_bias.x()));
    rec_.log(std::string(kImu) + "acc_bias/y",  rerun::Scalars(acc_bias.y()));
    rec_.log(std::string(kImu) + "acc_bias/z",  rerun::Scalars(acc_bias.z()));

    rec_.log(std::string(kImu) + "gyro_bias/x", rerun::Scalars(gyro_bias.x()));
    rec_.log(std::string(kImu) + "gyro_bias/y", rerun::Scalars(gyro_bias.y()));
    rec_.log(std::string(kImu) + "gyro_bias/z", rerun::Scalars(gyro_bias.z()));
}
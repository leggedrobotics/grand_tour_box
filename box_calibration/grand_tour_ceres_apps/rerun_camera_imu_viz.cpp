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

static rerun::Transform3D affineToRerunTransform(const Eigen::Affine3d& T) {
    const Eigen::Vector3d t = T.translation();
    const Eigen::Matrix3d R = T.rotation();
    return rerun::Transform3D::from_translation_mat3x3(
            {static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z())},
            rerun::Mat3x3({
                                  static_cast<float>(R(0,0)), static_cast<float>(R(1,0)), static_cast<float>(R(2,0)),
                                  static_cast<float>(R(0,1)), static_cast<float>(R(1,1)), static_cast<float>(R(2,1)),
                                  static_cast<float>(R(0,2)), static_cast<float>(R(1,2)), static_cast<float>(R(2,2)),
                          }));
}

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

void RerunCameraImuViz::vizRigPose3D(double timestamp_s, const Eigen::Affine3d& T_world_imu) {
    rec_.set_time_timestamp_secs_since_epoch(kTimeline, timestamp_s);
    rec_.log("world/rig",
             affineToRerunTransform(T_world_imu),
             rerun::TransformAxes3D(0.1f));
}

void RerunCameraImuViz::vizCameraPose3D(const std::string& camera_name,
                                        double timestamp_s,
                                        const Eigen::Affine3d& T_world_camera) {
    rec_.set_time_timestamp_secs_since_epoch(kTimeline, timestamp_s);
    rec_.log("world/camera_poses/" + camera_name,
             affineToRerunTransform(T_world_camera),
             rerun::TransformAxes3D(0.05f));
}

void RerunCameraImuViz::vizDetectionPoints3D(const std::string& camera_name,
                                             double timestamp_s,
                                             const Eigen::Affine3d& T_world_camera,
                                             const Eigen::Matrix3Xd& points_sensor) {
    if (points_sensor.cols() == 0) return;
    const Eigen::Matrix3Xd pts_world = T_world_camera * points_sensor;
    std::vector<std::array<float, 3>> positions(pts_world.cols());
    for (int i = 0; i < pts_world.cols(); ++i)
        positions[i] = {static_cast<float>(pts_world(0, i)),
                        static_cast<float>(pts_world(1, i)),
                        static_cast<float>(pts_world(2, i))};
    rec_.set_time_timestamp_secs_since_epoch(kTimeline, timestamp_s);
    rec_.log("world/camera_poses/" + camera_name + "_points",
             rerun::Points3D(positions).with_colors(colorForCamera(camera_name)).with_radii(0.01f));
}

void RerunCameraImuViz::vizBoardPose3D(const Eigen::Affine3d& T_world_board) {
    rec_.log_static("world/board",
                    affineToRerunTransform(T_world_board),
                    rerun::TransformAxes3D(0.2f));
}

void RerunCameraImuViz::vizWorldFramePointError(const std::string& camera_name,
                                                double timestamp_s,
                                                double mean_l2_error_metres) {
    const std::string path = std::string(kBase) + camera_name + "/world_frame_mean_point_error_m";
    rec_.log_static(path, rerun::SeriesLines()
            .with_colors(colorForCamera(camera_name)).with_widths(2));
    rec_.set_time_timestamp_secs_since_epoch(kTimeline, timestamp_s);
    rec_.log(path, rerun::Scalars(mean_l2_error_metres));
}

void RerunCameraImuViz::vizImuConsistencyResidual(double timestamp_s,
                                                   double position_norm,
                                                   double velocity_norm,
                                                   double rotation_norm) {
    static constexpr char kImuRes[] = "world/imu/consistency/";
    static const rerun::Color kColor{241, 194, 27};

    static bool styled = false;
    if (!styled) {
        styled = true;
        for (const char* path : {"world/imu/consistency/position",
                                  "world/imu/consistency/velocity",
                                  "world/imu/consistency/rotation"})
            rec_.log_static(path, rerun::SeriesLines().with_colors(kColor).with_widths(2));
    }

    rec_.set_time_timestamp_secs_since_epoch(kTimeline, timestamp_s);
    rec_.log(std::string(kImuRes) + "position", rerun::Scalars(position_norm));
    rec_.log(std::string(kImuRes) + "velocity", rerun::Scalars(velocity_norm));
    rec_.log(std::string(kImuRes) + "rotation", rerun::Scalars(rotation_norm));
}


void RerunCameraImuViz::vizExtrinsics(const std::map<std::string, Eigen::Affine3d>& T_bundle_cameras,
                                      const Eigen::Affine3d& T_bundle_imu) {
    static constexpr char kBundle[] = "world/bundle/";

    for (const auto& [camera_name, T] : T_bundle_cameras) {
        rec_.log_static(std::string(kBundle) + camera_name,
                        affineToRerunTransform(T), rerun::TransformAxes3D(.05f));
    }
    rec_.log_static(std::string(kBundle) + "imu",
                    affineToRerunTransform(T_bundle_imu), rerun::TransformAxes3D(.05f));
}

void RerunCameraImuViz::vizImuPoseTrajectory(const std::string& source,
                                             double timestamp_s,
                                             const Eigen::Affine3d& T_board_imu) {
    const std::string base = "imu_trajectory/" + source + "/position";
    const rerun::Color color = colorForCamera(source);
    // log_static is idempotent — safe to call every time.
    for (const std::string axis : {"/x", "/y", "/z"})
        rec_.log_static(base + axis, rerun::SeriesLines().with_colors(color).with_widths(2));

    rec_.set_time_timestamp_secs_since_epoch(kTimeline, timestamp_s);
    const Eigen::Vector3d t = T_board_imu.translation();
    rec_.log(base + "/x", rerun::Scalars(t.x()));
    rec_.log(base + "/y", rerun::Scalars(t.y()));
    rec_.log(base + "/z", rerun::Scalars(t.z()));
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
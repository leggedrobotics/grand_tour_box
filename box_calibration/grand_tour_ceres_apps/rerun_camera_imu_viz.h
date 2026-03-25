#pragma once

#include <unordered_map>
#include <rerun.hpp>
#include "camera_imu_viz_interface.h"

class RerunCameraImuViz : public CameraImuVizInterface {
public:
    explicit RerunCameraImuViz(rerun::RecordingStream& rec);

    void vizDetections(const std::string& camera_name,
                       double timestamp_s,
                       const std::vector<std::array<float, 2>>& corners_2d) override;

    void vizImage(const std::string& camera_name,
                  double timestamp_s,
                  const std::vector<uint8_t>& image_data,
                  uint32_t width,
                  uint32_t height) override;

    void vizCameraPose(const std::string& camera_name,
                       double timestamp_s,
                       const Eigen::Affine3d& T_camera_board) override;

    void vizRigPose3D(double timestamp_s, const Eigen::Affine3d& T_world_imu) override;
    void vizBoardPose3D(const Eigen::Affine3d& T_world_board) override;

    void vizReprojectionError(const std::string& camera_name,
                              double timestamp_s,
                              double rms_pixels) override;

    void vizImuConsistencyResidual(double timestamp_s,
                                   double position_norm,
                                   double velocity_norm,
                                   double rotation_norm) override;

    void vizExtrinsics(const std::map<std::string, Eigen::Affine3d>& T_bundle_cameras,
                       const Eigen::Affine3d& T_bundle_imu) override;

    void vizImuState(double timestamp_s,
                     const Eigen::Vector3d& position,
                     const Eigen::Vector3d& velocity,
                     const Eigen::Quaterniond& orientation,
                     const Eigen::Vector3d& acc_bias,
                     const Eigen::Vector3d& gyro_bias) override;

private:
    rerun::Color colorForCamera(const std::string& camera_name);

    rerun::RecordingStream& rec_;
    std::unordered_map<std::string, size_t> camera_color_index_;

    static constexpr char kTimeline[] = "camera_imu_timeline";
    static constexpr char kBase[] = "world/cameras/";
    static const std::vector<rerun::Color> kCameraColors;
};
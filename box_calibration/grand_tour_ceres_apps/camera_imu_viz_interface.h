#pragma once

#include <array>
#include <string>
#include <vector>
#include <Eigen/Geometry>

// Abstract viz interface for camera-IMU calibration.
// Pass as std::unique_ptr<CameraImuVizInterface> to the program.
class CameraImuVizInterface {
public:
    virtual ~CameraImuVizInterface() = default;

    // Visualize 2D april-grid corner detections for a single camera at a given time.
    // corners_2d: Nx2 array (col0=u, col1=v), in image pixel coordinates.
    virtual void vizDetections(const std::string& camera_name,
                               double timestamp_s,
                               const std::vector<std::array<float, 2>>& corners_2d) = 0;

    // Visualize a raw grayscale camera image at a given time.
    // image_data: row-major uint8 buffer, width * height bytes.
    virtual void vizImage(const std::string& camera_name,
                          double timestamp_s,
                          const std::vector<uint8_t>& image_data,
                          uint32_t width,
                          uint32_t height) = 0;

    // Log per-axis position (x, y, z) and orientation (roll, pitch, yaw) time series
    // for the camera pose T_camera_board at a given timestamp.
    // Each axis is a separate scalar series so all cameras share the same plot.
    virtual void vizCameraPose(const std::string& camera_name,
                               double timestamp_s,
                               const Eigen::Affine3d& T_camera_board) = 0;

    // Log integrated IMU state as separate per-axis time series:
    //   position (x/y/z), velocity (x/y/z), orientation (roll/pitch/yaw),
    //   accelerometer bias (x/y/z), gyroscope bias (x/y/z).
    virtual void vizImuState(double timestamp_s,
                             const Eigen::Vector3d& position,
                             const Eigen::Vector3d& velocity,
                             const Eigen::Quaterniond& orientation,
                             const Eigen::Vector3d& acc_bias,
                             const Eigen::Vector3d& gyro_bias) = 0;
};
#pragma once

#include <array>
#include <string>
#include <vector>
#include <map>
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

    // Log the IMU pose in world frame at a keyframe (time-varying — animates the rig trajectory).
    virtual void vizRigPose3D(double timestamp_s, const Eigen::Affine3d& T_world_imu) = 0;

    // Log a camera pose in world frame at a keyframe (time-varying — animates per-camera trajectory).
    virtual void vizCameraPose3D(const std::string& camera_name,
                                 double timestamp_s,
                                 const Eigen::Affine3d& T_world_camera) = 0;

    // Log sensor-frame 3D detection points projected into world frame at a keyframe.
    // points_sensor: 3×N matrix of points in the camera/sensor frame.
    virtual void vizDetectionPoints3D(const std::string& camera_name,
                                      double timestamp_s,
                                      const Eigen::Affine3d& T_world_camera,
                                      const Eigen::Matrix3Xd& points_sensor) = 0;

    // Log the board pose in world frame (static).
    virtual void vizBoardPose3D(const Eigen::Affine3d& T_world_board) = 0;

    // Log RMS world-frame 3D point error (metres) for a single camera at a keyframe.
    virtual void vizWorldFramePointError(const std::string& camera_name,
                                         double timestamp_s,
                                         double rms_metres) = 0;

    // Log norms of the IMU consistency sub-residuals for an interval starting at timestamp_s.
    virtual void vizImuConsistencyResidual(double timestamp_s,
                                           double position_norm,
                                           double velocity_norm,
                                           double rotation_norm,
                                           double bias_gyro_norm,
                                           double bias_accel_norm) = 0;

    // Log the static sensor extrinsics as 3D frames in the bundle coordinate system.
    // T_bundle_cameras: per-camera transform (sensor → bundle).
    // T_bundle_imu:     IMU transform (IMU → bundle).
    virtual void vizExtrinsics(const std::map<std::string, Eigen::Affine3d>& T_bundle_cameras,
                               const Eigen::Affine3d& T_bundle_imu) = 0;

    // Log XYZ position of an IMU pose as a time series under "imu_trajectory/<source>/".
    // Call with source="keyframe" for optimized keyframe poses and
    // source="camera_chain" for camera-detection-derived poses. Both land on the
    // same axes so they can be directly compared.
    virtual void vizImuPoseTrajectory(const std::string& source,
                                      double timestamp_s,
                                      const Eigen::Affine3d& T_board_imu) = 0;

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
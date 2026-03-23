#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Abstract viz interface for camera-prism calibration.
// Pass as std::unique_ptr<CameraPrismVizInterface> to the program via setViz().
class CameraPrismVizInterface {
public:
    virtual ~CameraPrismVizInterface() = default;

    // Log static world frames: the total station is the world origin;
    // T_leica_board is the pose of the calibration board in the leica/world frame.
    virtual void vizStaticFrames(const Eigen::Affine3d& T_leica_board) = 0;

    // Prism trajectory as measured by the total station: a single 3D line strip
    // and per-axis (x/y/z) scalar time series.
    // timestamps_s: observation times in seconds. positions: (x,y,z) in leica frame.
    // Positions must be supplied in chronological order.
    virtual void vizPrismObservedTrajectory(
        const std::vector<double>& timestamps_s,
        const std::vector<std::array<float, 3>>& positions) = 0;

    // Prism positions from the total station, linearly interpolated to each camera
    // detection timestamp. Same frame and format as vizPrismObservedTrajectory.
    // Positions must be supplied in chronological order.
    virtual void vizPrismInterpolatedAtCameraTime(
        const std::vector<double>& timestamps_s,
        const std::vector<std::array<float, 3>>& positions) = 0;

    // Estimated prism position in the leica frame, derived from camera board detections
    // and the current calibration (T_leica_board @ T_board_cam @ T_cam_cam0 @ t_cam0_prism).
    // Each camera is rendered in a distinct colour in the concrete implementation.
    // Positions must be supplied in chronological order.
    virtual void vizPrismEstimatedByCameraDetections(
        const std::string& camera_name,
        const std::vector<double>& timestamps_s,
        const std::vector<std::array<float, 3>>& positions) = 0;

    // Camera trajectory in the leica/world frame over time, derived from board detections:
    // T_leica_cam(t) = T_leica_board @ T_board_cam(t).
    // Logged as a 3D line strip of camera centre positions and per-axis scalar time series.
    // Also logs a Transform3D at each timestamp so camera orientation is visible on the timeline.
    // Entries must be supplied in chronological order.
    virtual void vizCameraTrajectory(
        const std::string& camera_name,
        const std::vector<double>& timestamps_s,
        const std::vector<Eigen::Affine3d>& T_leica_cam_over_time) = 0;

    // Extrinsics-only calibration sigmas: rt_sigmas maps camera name to a 6-vector
    // (rx, ry, rz, tx, ty, tz) of parameter standard deviations from the solver.
    // origin_camera is fixed at the origin and always has zero sigmas.
    virtual void vizCalibrationSigmas(
        const std::map<std::string, Eigen::VectorXd>& rt_sigmas,
        const std::string& origin_camera,
        double translation_threshold = 0.001,
        double rotation_threshold = 0.001) = 0;
};
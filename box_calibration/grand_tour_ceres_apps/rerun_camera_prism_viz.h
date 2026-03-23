#pragma once

#include <unordered_map>
#include <rerun.hpp>
#include "camera_prism_viz_interface.h"

class RerunCameraPrismViz : public CameraPrismVizInterface {
public:
    explicit RerunCameraPrismViz(rerun::RecordingStream& rec);

    void vizStaticFrames(const Eigen::Affine3d& T_leica_board) override;

    void vizPrismObservedTrajectory(
        const std::vector<double>& timestamps_s,
        const std::vector<std::array<float, 3>>& positions) override;

    void vizPrismInterpolatedAtCameraTime(
        const std::vector<double>& timestamps_s,
        const std::vector<std::array<float, 3>>& positions) override;

    void vizPrismEstimatedByCameraDetections(
        const std::string& camera_name,
        const std::vector<double>& timestamps_s,
        const std::vector<std::array<float, 3>>& positions) override;

    void vizCameraTrajectory(
        const std::string& camera_name,
        const std::vector<double>& timestamps_s,
        const std::vector<Eigen::Affine3d>& T_leica_cam_over_time) override;

    void vizCalibrationSigmas(
        const std::map<std::string, Eigen::VectorXd>& rt_sigmas,
        const std::string& origin_camera,
        double translation_threshold = 0.001,
        double rotation_threshold = 0.001) override;

private:
    // Returns a stable colour for a given camera name, assigned on first use.
    rerun::Color colorForCamera(const std::string& camera_name);

    // Log a 3D line strip (timeless) and per-axis scalar time series for a set of positions.
    // The strip is logged at strip_entity.
    // Scalars are logged at series_parent/x/series_name, .../y/..., .../z/...
    // so that multiple sources sharing the same series_parent appear on the same plot.
    void logStripAndTimeSeries(const std::string& strip_entity,
                               const std::string& series_parent,
                               const std::string& series_name,
                               const rerun::Color& color,
                               const std::vector<double>& timestamps_s,
                               const std::vector<std::array<float, 3>>& positions);

    rerun::RecordingStream& rec_;
    std::unordered_map<std::string, size_t> camera_color_index_;

    static constexpr char base_name_[] = "world/";
    static constexpr char calibration_sigmas_preamble_[] =
        "# Grand Tour Cam-Prism Calibration Viz\n"
        "Per-parameter standard deviations from the solver.\n\n"
        "**Sigma table:**\n"
        "- ~~Strikethrough~~ — insufficient data for this camera\n\n"
        "- Plain — exceeds threshold (collect more data)\n"
        "- **Bold** — within threshold (good)\n\n"
        "cam0 is the origin, so it has no extrinsic sigmas\n\n";
    static const std::vector<rerun::Color> kCameraColors;
};
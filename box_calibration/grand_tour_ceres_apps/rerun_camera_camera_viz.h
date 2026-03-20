//
// Created by fu on 18/03/26.
//

#ifndef GRAND_TOUR_CERES_APPS_RERUN_CAMERA_CAMERA_VIZ_H
#define GRAND_TOUR_CERES_APPS_RERUN_CAMERA_CAMERA_VIZ_H

#include <chrono>
#include <unordered_map>
#include <rerun.hpp>
#include "camera_camera_viz_interface.h"

class RerunCameraCameraViz : public CameraCameraVizInterface {
public:
    explicit RerunCameraCameraViz(rerun::RecordingStream& rec,
                                  std::chrono::milliseconds throttle_ms = std::chrono::milliseconds(1000));

    void vizDetections(const std::string& view_name,
                       const std::vector<std::array<float, 2>>& corners_2d) override;

    void vizCovariances(const std::string& view_name,
                        const Eigen::VectorXf& covariances) override;

    void vizVoxelMap(const std::string &view_name, const std::vector<std::array<int32_t, 2>> &coords,
                     const std::vector<uint32_t> &counts, float voxel_size, int width, int height) override;

    void vizFrameTransforms(const std::map<std::string, Eigen::Affine3d>& transforms) override;

    void vizResidualMap(const std::string& view_name,
                        const std::vector<std::array<float, 2>>& observations,
                        const std::vector<float>& residual_magnitudes,
                        int width, int height) override;

    void vizAdjacencyGraph(const std::string& view_name,
                           const std::vector<std::string>& nodes,
                           const std::vector<std::pair<std::string, std::string>>& edges,
                           const std::vector<int>& edge_counts) override;

    void vizDataAccumulationProgress(float percentage) override;

    void vizCalibrationSigmasPreamble() override;

    void vizAllCameraCalibrationSigmas(
            const std::map<std::string, std::pair<Eigen::VectorXd, Eigen::VectorXd>>& sigmas,
            const std::string& origin_camera,
            double intrinsics_threshold = 1.0,
            double translation_threshold = 0.001,
            double rotation_threshold = 0.001) override;

private:
    bool shouldLog(const std::string& view_name);

    rerun::RecordingStream& rec_;
    std::chrono::milliseconds throttle_ms_;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_log_time_;
    static constexpr char base_name_[] = "world/";
    static constexpr char calibration_sigmas_preamble_[] =
        "# Grand Tour Cam Calibration Viz\n"
        "Per-parameter standard deviations from the solver.\n\n"
        "**Sigma table:**\n"
        "- ~~Strikethrough~~ — insufficient data for this camera\n\n"
        "- Plain — exceeds threshold (collect more data)\n"
        "- **Bold** — within threshold (good)\n\n"
        "**Data collection bar:**\n"
        "- Green — keep collecting data\n"
        "- Red — optimization running, pause and wait\n\n"
        "cam1_sensor_frame is the origin, so it has no extrinsic sigmas\n\n";
};

#endif //GRAND_TOUR_CERES_APPS_RERUN_CAMERA_CAMERA_VIZ_H
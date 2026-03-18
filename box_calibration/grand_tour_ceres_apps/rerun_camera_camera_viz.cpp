//
// Created by fu on 18/03/26.
//

#include "rerun_camera_camera_viz.h"

RerunCameraCameraViz::RerunCameraCameraViz(rerun::RecordingStream& rec,
                                           std::chrono::milliseconds throttle_ms)
    : rec_(rec), throttle_ms_(throttle_ms) {}

bool RerunCameraCameraViz::shouldLog(const std::string& view_name) {
    const auto now = std::chrono::steady_clock::now();
    auto& last = last_log_time_[view_name];
    if (now - last < throttle_ms_) return false;
    last = now;
    return true;
}

void RerunCameraCameraViz::vizDetections(const std::string& view_name,
                                         const std::vector<std::array<float, 2>>& corners_2d) {
    if (!shouldLog(view_name)) return;
    rec_.log(view_name, rerun::Points2D(corners_2d));
}

void RerunCameraCameraViz::vizCovariances(const std::string& view_name,
                                           const Eigen::VectorXf& covariances) {
    if (!shouldLog(view_name)) return;
    (void)covariances;
}

void RerunCameraCameraViz::vizVoxelMap(const std::string &view_name, const std::vector<std::array<int32_t, 2>> &coords,
                                       const std::vector<uint32_t> &counts, float voxel_size, int width, int height) {
    if (coords.empty() || !shouldLog(view_name)) return;

    const auto vs = static_cast<int32_t>(voxel_size);

    // Image dimensions in original pixel space
    const int32_t img_width  = width;
    const int32_t img_height = height;

    std::vector<float> grid(static_cast<size_t>(img_width * img_height), 0.0f);
    for (size_t i = 0; i < coords.size(); ++i) {
        const int32_t col_origin = coords[i][0] * vs;
        const int32_t row_origin = coords[i][1] * vs;
        const float count = static_cast<float>(counts[i]);
        for (int32_t dy = 0; dy < vs; ++dy) {
            for (int32_t dx = 0; dx < vs; ++dx) {
                grid[static_cast<size_t>((row_origin + dy) * img_width + (col_origin + dx))] = count;
            }
        }
    }

    rec_.log(view_name,
             rerun::DepthImage(grid.data(), {static_cast<uint32_t>(img_width), static_cast<uint32_t>(img_height)}));
}
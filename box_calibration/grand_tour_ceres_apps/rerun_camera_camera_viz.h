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

private:
    bool shouldLog(const std::string& view_name);

    rerun::RecordingStream& rec_;
    std::chrono::milliseconds throttle_ms_;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_log_time_;
};

#endif //GRAND_TOUR_CERES_APPS_RERUN_CAMERA_CAMERA_VIZ_H
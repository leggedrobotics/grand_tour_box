//
// Created by fu on 18/03/26.
//

#ifndef GRAND_TOUR_CERES_APPS_CAMERA_CAMERA_VIZ_INTERFACE_H
#define GRAND_TOUR_CERES_APPS_CAMERA_CAMERA_VIZ_INTERFACE_H

#include <array>
#include <string>
#include <vector>
#include <Eigen/Core>

// Abstract viz interface for camera-camera calibration.
// Pass as std::unique_ptr<CameraCameraVizInterface> to offline/online programs.
class CameraCameraVizInterface {
public:
    virtual ~CameraCameraVizInterface() = default;

    // Visualize 2D corner detections. corners_2d: Nx2 matrix (col0=u, col1=v).
    virtual void vizDetections(const std::string& view_name,
                               const std::vector<std::array<float, 2>>& corners_2d) = 0;

    // Visualize per-corner covariance diagonals. covariances: N*2 vector (uu, vv per corner).
    virtual void vizCovariances(const std::string& view_name,
                                const Eigen::VectorXf& covariances) = 0;

    // Visualize a 2D voxel occupancy map as a density image.
    // coords: voxel grid indices (integer pixel-space / voxel_size).
    // counts: occupancy count per voxel, parallel to coords.
    // voxel_size: size of each voxel in pixels, used to place the image correctly.
    virtual void vizVoxelMap(const std::string &view_name, const std::vector<std::array<int32_t, 2>> &coords,
                             const std::vector<uint32_t> &counts, float voxel_size, int width, int height) = 0;
};

#endif //GRAND_TOUR_CERES_APPS_CAMERA_CAMERA_VIZ_INTERFACE_H
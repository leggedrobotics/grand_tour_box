//
// Created by fu on 18/03/26.
//

#ifndef GRAND_TOUR_CERES_APPS_CAMERA_CAMERA_VIZ_INTERFACE_H
#define GRAND_TOUR_CERES_APPS_CAMERA_CAMERA_VIZ_INTERFACE_H

#include <array>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <map>
#include <Eigen/Geometry>

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

    // Visualize camera frame transforms in 3D space.
    // transforms: map from camera name to T_bundle_sensor as Affine3d.
    // Each camera is logged as a child entity under view_name.
    virtual void vizFrameTransforms(const std::map<std::string, Eigen::Affine3d>& transforms) = 0;

    // Visualize reprojection residuals as a 2D scatter plot centred at the origin.
    // residuals_2d: (dx, dy) reprojection error vectors, one per observation.
    virtual void vizResidualMap(const std::string& view_name,
                                const std::vector<std::array<float, 2>>& residuals_2d) = 0;

    // Visualize the camera-camera adjacency graph.
    // nodes: camera names (graph node IDs and labels).
    // edges: pairs of (source, target) camera names.
    // edge_counts: co-detection count per edge, parallel to edges.
    virtual void vizAdjacencyGraph(const std::string& view_name,
                                   const std::vector<std::string>& nodes,
                                   const std::vector<std::pair<std::string, std::string>>& edges,
                                   const std::vector<int>& edge_counts) = 0;

    // Visualize data accumulation progress as a bar chart with y-axis range [0, 1].
    // percentage: fraction of data collected, clamped to [0, 1].
    virtual void vizDataAccumulationProgress(float percentage) = 0;

    // Log the calibration sigma preamble (legend + bar chart hint) to the sigmas panel.
    // Call this before the optimisation starts so the panel is populated immediately.
    virtual void vizCalibrationSigmasPreamble() = 0;

    // Visualize calibration sigmas for all cameras as three markdown tables (TextDocument):
    //   intrinsics (fx fy cx cy), translation (tx ty tz), rotation (rx ry rz).
    // sigmas: map from camera name → {rtvec_sigma (6), fxfycxcy_sigma (4)}.
    // origin_camera: camera fixed at the origin — always has zero rt sigmas, never struck through.
    // Row states — intrinsics: regular → ✓ bold (converged).
    //              translation/rotation: ~~strikethrough~~ (all zeros, non-origin) → regular → ✓ bold.
    virtual void vizAllCameraCalibrationSigmas(
            const std::map<std::string, std::pair<Eigen::VectorXd, Eigen::VectorXd>>& sigmas,
            const std::string& origin_camera,
            double intrinsics_threshold = 1.0,
            double translation_threshold = 0.001,
            double rotation_threshold = 0.001) = 0;
};

#endif //GRAND_TOUR_CERES_APPS_CAMERA_CAMERA_VIZ_INTERFACE_H
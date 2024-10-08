//
// Created by fu on 06/10/24.
//

#include <gtboxcalibration/voxel_2d.h>

VoxelMap2D::VoxelMap2D(double voxel_size_in_pixels) : voxel_size_(voxel_size_in_pixels) {}

bool VoxelMap2D::addToMap(const Eigen::Matrix2Xd &points) {
    if (points.minCoeff() < 0) {
        return false;
    }

    const auto voxels = computeVoxels(points);
    for (const auto &voxel: voxels) {
        data_[voxel]++;
    }
    return true;
}

std::vector<Voxel2D> VoxelMap2D::computeVoxels(const Eigen::Matrix2Xd &points) const {
    std::vector<Voxel2D> voxels;
    voxels.reserve(points.cols());
    for (int i = 0; i < points.cols(); ++i) {
        // Get the x and y coordinates of the point
        double x = points(0, i);
        double y = points(1, i);

        // Convert 2D point coordinates to voxel indices
        int x_voxel = floor(x / voxel_size_);
        int y_voxel = floor(y / voxel_size_);

        Voxel2D voxel{x_voxel, y_voxel};
        voxels.push_back(voxel);
    }
    return voxels;
}

bool VoxelMap2D::addToMapIfAnyIsBelowCapacity(const Eigen::Matrix2Xd &points, int max_capacity) {
    if (points.minCoeff() < 0) return false;

    const auto voxels = computeVoxels(points);
    const bool do_add = std::any_of(voxels.begin(), voxels.end(),
                                              [max_capacity, this](const auto voxel) {
                                                  return data_[voxel] < max_capacity;
                                              });
    if (!do_add) return false;
    for (const auto& voxel : voxels) {
        data_[voxel]++;
    }
    return true;
}

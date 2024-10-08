//
// Created by fu on 06/10/24.
//

#ifndef GRAND_TOUR_CERES_APPS_VOXEL_2_D_H
#define GRAND_TOUR_CERES_APPS_VOXEL_2_D_H

#include <iostream>
#include <unordered_map>
#include <Eigen/Dense>
#include <cmath>

// Define a custom structure for 2D voxel grid coordinates
struct Voxel2D {
    int x, y;

    bool operator==(const Voxel2D &other) const {
        return x == other.x && y == other.y;
    }
};

// Custom hash function for Voxel2D
struct Voxel2DHash {
    std::size_t operator()(const Voxel2D &v) const {
        return std::hash<int>()(v.x) ^ std::hash<int>()(v.y);
    }
};

struct VoxelMap2D {
    VoxelMap2D() = default;
    explicit VoxelMap2D(double voxel_size_in_pixels);

    bool addToMap(const Eigen::Matrix2Xd &points);

    std::vector<Voxel2D> computeVoxels(const Eigen::Matrix2Xd &points) const;

    bool addToMapIfAnyIsBelowCapacity(const Eigen::Matrix2Xd &points, int max_capacity);

    double voxel_size_;
    std::unordered_map<Voxel2D, unsigned long, Voxel2DHash> data_;
};


#endif //GRAND_TOUR_CERES_APPS_VOXEL_2_D_H

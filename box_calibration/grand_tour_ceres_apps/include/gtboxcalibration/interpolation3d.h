//
// Created by fu on 10/09/2024.
//

#ifndef GRAND_TOUR_CERES_APPS_INTERPOLATION3D_H
#define GRAND_TOUR_CERES_APPS_INTERPOLATION3D_H

#include <Eigen/Dense>
#include <map>
#include <optional>
#include <tuple>

// Function to perform linear interpolation between two vectors
template<typename T>
Eigen::Matrix<T, 3, 1> lerp(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2,
                            const T t) {
    assert(0 <= t && t <= 1);
    return (1.0 - t) * v1 + t * v2;
}

template<typename Key, typename V>
std::optional<std::pair<Key, Key>> findLeftRightTimeBounds(const std::map<Key, V> &detections, Key key) {
    // Finding the first element strictly smaller than the key
    auto it_upper = detections.upper_bound(key);  // First element > key
    auto it_lower = detections.lower_bound(key);  // First element >= key

    // Move it_lower back to find the element strictly smaller
    if (it_lower != detections.begin()) {
        --it_lower;
    }

    if (it_lower!= detections.begin() and it_upper != detections.end()) {
        return {std::make_pair(it_lower->first, it_upper->first)};
    }

    return {};
}

#endif //GRAND_TOUR_CERES_APPS_INTERPOLATION3D_H

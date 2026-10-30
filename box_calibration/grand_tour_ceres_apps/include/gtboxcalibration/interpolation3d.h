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
    if (detections.size() < 2 || key < detections.begin()->first || key > detections.rbegin()->first)
        return {};
    auto upper = detections.upper_bound(key);
    if (upper == detections.end()) upper = std::prev(detections.end());
    const auto lower = std::prev(upper);
    return std::make_pair(lower->first, upper->first);

}

#endif //GRAND_TOUR_CERES_APPS_INTERPOLATION3D_H

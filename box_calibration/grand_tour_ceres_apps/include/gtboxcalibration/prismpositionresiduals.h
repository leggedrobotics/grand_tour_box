//
// Created by fu on 09/09/2024.
//

#ifndef COMPUTE_CONNECTIVITY_PRISMPOSITIONRESIDUALS_H
#define COMPUTE_CONNECTIVITY_PRISMPOSITIONRESIDUALS_H


#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <vector>
#include <gtboxcalibration/camerageometry.h>
#include "datatypes.h"
#include "interpolation3d.h"

// Select a real interpolation interval. Unsupported samples must never become
// zero-cost residuals: that would reward a time offset that leaves the data.
inline auto prismTimeBounds(const PrismPositionDetectionData& data,
                            unsigned long long stamp, double offset) {
    using Bounds = std::optional<std::pair<unsigned long long, unsigned long long>>;
    const long double corrected = static_cast<long double>(stamp) + offset * 1e9L;
    if (!std::isfinite(offset) || corrected < 0 ||
        corrected > std::numeric_limits<unsigned long long>::max()) return Bounds{};
    auto bounds = findLeftRightTimeBounds(data, static_cast<unsigned long long>(corrected));
    if (!bounds || bounds->second <= bounds->first || bounds->second - bounds->first > 100000000ull)
        return Bounds{};
    return bounds;
}

template<typename T> double prismScalar(const T& value) { return static_cast<double>(value); }
template<typename T, int N> double prismScalar(const ceres::Jet<T, N>& value) { return value.a; }

double prismResidualMedian(std::vector<double> values);

class PrismInCam0InBoardInTotalStationConsistencyError {
public:
    PrismInCam0InBoardInTotalStationConsistencyError(const Eigen::Affine3d &tCamselfCam0,
                                                     const Eigen::Affine3d &tBoardCamself,
                                                     const unsigned long long camera_detection_time,
                                                     const PrismPositionDetectionData& prism_detections,
                                                     double prism_sigma = 0.003);

    template<typename T>
    bool operator()(
            const T *const params_T_leica_board,
            const T *const params_t_cam0_prism,
            const T *const params_t_offset_sec,
            T *residual) const {
        const auto left_right_timebound = prismTimeBounds(
                total_station_prism_measurement_data_, camera_detection_time_, prismScalar(params_t_offset_sec[0]));
        if (!left_right_timebound) return false;
        const auto time_segment_nsec = left_right_timebound->second - left_right_timebound->first;
        // Subtract epoch timestamps before converting to double to retain precision.
        const double relative_nsec = static_cast<double>(static_cast<long double>(camera_detection_time_) -
                                                         left_right_timebound->first);
        T alpha = (T(relative_nsec) + params_t_offset_sec[0] * 1e9) / double(time_segment_nsec);
        Eigen::Matrix<T, 3, 1> total_station_measured_point = lerp(
                total_station_prism_measurement_data_.at(left_right_timebound->first),
                total_station_prism_measurement_data_.at(left_right_timebound->second),
                alpha);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_cam0_prism(params_t_cam0_prism);
        Eigen::Matrix<T, 3, -1> prism_position_in_board = T_board_camself_ * T_camself_cam0_ * t_cam0_prism;

        Eigen::Matrix<T, 3, -1> prism_position_predicted =
                se3_transform_(params_T_leica_board, prism_position_in_board);

        // sigma is a standard deviation in metres, not a variance.
        const double M_1_sqrt_totalstation_cov = 1. / prism_sigma_;
        Eigen::Matrix<T, 3, -1> errors = (total_station_measured_point - prism_position_predicted) * M_1_sqrt_totalstation_cov;
        residual[0] = errors(0, 0);
        residual[1] = errors(1, 0);
        residual[2] = errors(2, 0);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Affine3d &tCamselfCam0,
                                       const Eigen::Affine3d &tBoardCamself,
                                       const unsigned long long camera_detection_time,
                                       const PrismPositionDetectionData& prism_detections,
                                       double prism_sigma = 0.003) {
        return new ceres::AutoDiffCostFunction<PrismInCam0InBoardInTotalStationConsistencyError,
                3,
                SE3Transform::NUM_PARAMETERS, 3, 1>(
                new PrismInCam0InBoardInTotalStationConsistencyError(tCamselfCam0,
                                                                     tBoardCamself,
                                                                     camera_detection_time,
                                                                     prism_detections, prism_sigma)
        );
    }

private:
    Eigen::Affine3d T_camself_cam0_;
    Eigen::Affine3d T_board_camself_;
    SE3Transform se3_transform_;
    double prism_sigma_;
    const unsigned long long camera_detection_time_;
    const PrismPositionDetectionData& total_station_prism_measurement_data_;
//    T_leica_board @ T_board_camself @ T_camself_cam0 @ t_cam0_prism = total_station_measured_point;
};

std::pair<Eigen::VectorXd, double> GetResidualsAndLossAtResidualBlockID(const ceres::Problem &problem,
                                                                        ceres::ResidualBlockId residualBlockId);


#endif //COMPUTE_CONNECTIVITY_PRISMPOSITIONRESIDUALS_H

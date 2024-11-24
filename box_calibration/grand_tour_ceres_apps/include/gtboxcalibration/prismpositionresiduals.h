//
// Created by fu on 09/09/2024.
//

#ifndef COMPUTE_CONNECTIVITY_PRISMPOSITIONRESIDUALS_H
#define COMPUTE_CONNECTIVITY_PRISMPOSITIONRESIDUALS_H


#include <Eigen/Dense>
#include <gtboxcalibration/camerageometry.h>
#include "datatypes.h"
#include "interpolation3d.h"

// Generic templated function
template<typename T>
long long castToLL(const T &value) {
    // If T is not a Jet, directly cast to unsigned long long
    return static_cast<unsigned long long>(value);
}

// Specialization for ceres::Jet
template<typename T, int N>
long long castToLL(const ceres::Jet<T, N> &jet) {
    // Extract the scalar part (jet.a) and cast it to unsigned long long
    return static_cast<unsigned long long>(jet.a);
}

class PrismInCam0InBoardInTotalStationConsistencyError {
public:
    PrismInCam0InBoardInTotalStationConsistencyError(const Eigen::Affine3d &tCamselfCam0,
                                                     const Eigen::Affine3d &tBoardCamself,
                                                     const unsigned long long camera_detection_time,
                                                     const PrismPositionDetectionData& prism_detections);

    template<typename T>
    bool operator()(
            const T *const params_T_leica_board,
            const T *const params_t_cam0_prism,
            const T *const params_t_offset_sec,
            T *residual) const {
        // The look up is not differentiable.
        double t_offset_nsec = castToLL(params_t_offset_sec[0] * 1e9);
        auto camera_stamp_corrected_lookup = static_cast<unsigned long long> (camera_detection_time_ +
                                                                                            t_offset_nsec);
        const auto left_right_timebound = findLeftRightTimeBounds(total_station_prism_measurement_data_,
                                                                  camera_stamp_corrected_lookup);
        for (int i = 0; i < 3; i++){
            residual[i] = T(0.0);
        }
        if (!left_right_timebound.has_value()) {
            return true;
        }
        unsigned long long time_segment_nsec = left_right_timebound->second - left_right_timebound->first;
        if (time_segment_nsec > 100000000ull) { // greater than 0.1 seconds. Prism is going at 20Hz btw
            return true;
        }

        T camera_stamp_corrected = double(camera_detection_time_) + params_t_offset_sec[0] * 1e9;
        T start_to_camera_stamp_corrected = camera_stamp_corrected - T(left_right_timebound->first);
        T alpha = start_to_camera_stamp_corrected / double(time_segment_nsec);
        Eigen::Matrix<T, 3, 1> total_station_measured_point = lerp(
                total_station_prism_measurement_data_.at(left_right_timebound->first),
                total_station_prism_measurement_data_.at(left_right_timebound->second),
                alpha);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_cam0_prism(params_t_cam0_prism);
        Eigen::Matrix<T, 3, -1> prism_position_in_board = T_board_camself_ * T_camself_cam0_ * t_cam0_prism;

        Eigen::Matrix<T, 3, -1> prism_position_predicted =
                se3_transform_(params_T_leica_board, prism_position_in_board);

        constexpr double M_1_sqrt_totalstation_cov = 1. / std::sqrt(0.003);
        Eigen::Matrix<T, 3, -1> errors = (total_station_measured_point - prism_position_predicted) * M_1_sqrt_totalstation_cov;
        residual[0] = errors(0, 0);
        residual[1] = errors(1, 0);
        residual[2] = errors(2, 0);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Affine3d &tCamselfCam0,
                                       const Eigen::Affine3d &tBoardCamself,
                                       const unsigned long long camera_detection_time,
                                       const PrismPositionDetectionData& prism_detections) {
        return new ceres::AutoDiffCostFunction<PrismInCam0InBoardInTotalStationConsistencyError,
                3,
                SE3Transform::NUM_PARAMETERS, 3, 1>(
                new PrismInCam0InBoardInTotalStationConsistencyError(tCamselfCam0,
                                                                     tBoardCamself,
                                                                     camera_detection_time,
                                                                     prism_detections)
        );
    }

private:
    Eigen::Affine3d T_camself_cam0_;
    Eigen::Affine3d T_board_camself_;
    SE3Transform se3_transform_;
    const unsigned long long camera_detection_time_;
    const PrismPositionDetectionData& total_station_prism_measurement_data_;
//    T_leica_board @ T_board_camself @ T_camself_cam0 @ t_cam0_prism = total_station_measured_point;
};

std::pair<Eigen::VectorXd, double> GetResidualsAndLossAtResidualBlockID(const ceres::Problem &problem,
                                                                        ceres::ResidualBlockId residualBlockId);


#endif //COMPUTE_CONNECTIVITY_PRISMPOSITIONRESIDUALS_H

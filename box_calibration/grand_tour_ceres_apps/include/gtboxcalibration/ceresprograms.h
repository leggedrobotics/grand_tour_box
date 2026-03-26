//
// Created by fu on 28/08/2024.
//

#ifndef COMPUTE_CONNECTIVITY_CERESBUILDERS_H
#define COMPUTE_CONNECTIVITY_CERESBUILDERS_H

#include <ceres/ceres.h>
#include <map>
#include <string>
#include <gtboxcalibration/argparsers.h>
#include <gtboxcalibration/datatypes.h>
#include <gtboxcalibration/parameterhelpers.h>
#include <gtboxcalibration/reprojectionresiduals.h>
#include <gtboxcalibration/utils.h>
#include <gtboxcalibration/ceresproblems.h>


struct CeresProgram {
    virtual bool PopulateProblem() = 0;

    std::unique_ptr<CeresProblem> problem_;

    CeresProgram();

    virtual bool Solve();

    void ResetAndRepopulateProblem();

    void PrintParameterAndResidualBlockStats();
};

CameraCamera2D3DTargetDetectionData
FixCameraDetectionTimes(const CameraCamera2D3DTargetDetectionData &input_camera_detections,
                        const PrismPositionDetectionData &prism_detections);

struct CameraIMUProgram : CeresProgram {
    CameraIMUProgram() = default;

    bool PopulateProblem() override;
    void PreSolveExtrinsic();
    void PreSolveBoard();

    // --- Observations ---
    std::map<std::string, CameraParameterPack> camera_packs;
    CameraCamera2D3DTargetDetectionData camera_detections;
    IMUObservationData imu_observations;

    // --- Parameters: global ---
    double T_camera_bundle_imu[SE3Transform::NUM_PARAMETERS]{0, 0, 0, 1, 0, 0, 0};
    double T_world_board[SE3Transform::NUM_PARAMETERS]{0, 0, 0, 1, 0, 0, 0};
    Eigen::Vector3d gravity_world{0.0, 0.0, -9.81};

    // Global bias shared across all residuals.
    double bias_gyro[3]{};
    double bias_accel[3]{};

    // --- Parameters: per-keyframe (velocity only — T_world_imu derived from chain) ---
    std::map<unsigned long long, ImuKeyframeParameterPack> keyframe_params;

    // --- Residual block tracking ---
    std::map<unsigned long long, ceres::ResidualBlockId> relative_residual_block_map;

    // --- Metadata ---
    std::string cam0_name;
    std::map<std::string, Eigen::Affine3d> T_bundle_cam;
    std::string output_yaml_path;
    std::string cameras_calibration_path;
    bool solve_time_offset = false;
};

struct CameraPrismProgram : CeresProgram {
    CameraPrismProgram() = default;
    explicit CameraPrismProgram(CameraPrismCalibrationAppParser argparser);

    bool PopulateProblem() override;

    void WriteOutputParameters();

    CameraCamera2D3DTargetDetectionData camera_detections;
    std::map<std::string, Eigen::Affine3d> T_bundle_cam;
    std::string cam0_name;
    PrismPositionDetectionData prism_detections;
    PrismBoardInTotalStationParameterPack prism_board_in_total_station_params;
    std::map<std::string, std::map<unsigned long long, ceres::ResidualBlockId >> residual_block_map;
    std::string output_yaml_path;
    std::string cameras_calibration_path;
    bool solve_time_offset = false;
    unsigned long long calibration_time_nsec;
    static constexpr double prism_sigma_ = 0.003;
    static constexpr double prism_sigma2_ = prism_sigma_ * prism_sigma_;
};

struct CameraCameraProgram : CeresProgram {
    explicit CameraCameraProgram(CameraCameraCalibrationAppParser);

    CameraCameraProgram();

    bool PopulateProblem() override;

    void SetPresolveCameraExtrinsicsConstants();

    std::map<std::string, CameraParameterPack> camera_parameter_packs;
    std::map<std::string, bool> has_initialised_extrinsics;
    std::map<std::string, std::map<unsigned long long, std::shared_ptr<BoardPoseParameterPack>>> board_pose_parameter_packs;
    CameraCamera2D3DTargetDetectionData parsed_alignment_data;
    std::map<std::string, std::map<unsigned long long, ceres::ResidualBlockId>> intrinsics_residuals_of_camera_at_time;
    std::map<std::string,
            std::map<std::string,
            std::map<unsigned long long, ceres::ResidualBlockId>>> extrinsics_residuals_of_cameras_at_time;
    std::map<std::string, std::map<std::string, int>> camera_camera_adjacency_count;
    std::string origin_camera_frame_id;

    ceres::ResidualBlockId
    addBoardPoseParameterAndCameraIntrinsicsResidualFromObservation(const unsigned long long stamp,
                                                                    const Observations2dModelPoints3dPointIDsPose3dSensorName &observation_i);

    ceres::ResidualBlockId
    addExtrinsicResidualFromObservations(const unsigned long long stamp_i,
                                         const unsigned long long stamp_j,
                                         const Observations2dModelPoints3dPointIDsPose3dSensorName &observation_i,
                                         const Observations2dModelPoints3dPointIDsPose3dSensorName &observation_j,
                                         const std::vector<unsigned int> &common_ids);
};

void populateStereoProjectionProblem(ceres::Problem &problem,
                                     std::map<std::string, CameraParameterPack> &camera_parameter_packs,
                                     std::map<std::string, std::map<unsigned long long, BoardPoseParameterPack>>
                                     &board_pose_parameter_packs,
                                     const CameraCamera2D3DTargetDetectionData &parsed_alignment_data,
                                     std::map<std::string, std::map<unsigned long long, ceres::ResidualBlockId>> &residual_block_id_of_camera_at_time);


#endif //COMPUTE_CONNECTIVITY_CERESBUILDERS_H

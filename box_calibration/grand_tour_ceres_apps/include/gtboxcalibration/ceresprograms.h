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

    bool Solve();

    void ResetAndRepopulateProblem();

    void PrintParameterAndResidualBlockStats();
};

CameraCamera2D3DTargetDetectionData
FixCameraDetectionTimes(const CameraCamera2D3DTargetDetectionData &input_camera_detections,
                        const PrismPositionDetectionData &prism_detections);

struct CameraPrismProgram : CeresProgram {
    CameraPrismProgram(CameraPrismCalibrationAppParser argparser);

    bool PopulateProblem() override;

    CameraCamera2D3DTargetDetectionData camera_detections;
    std::map<std::string, Eigen::Affine3d> T_bundle_cam;
    std::string cam0_name;
    PrismPositionDetectionData prism_detections;
    PrismBoardInTotalStationParameterPack prism_board_in_total_station_params;
    std::map<std::string, std::map<unsigned long long, ceres::ResidualBlockId >> residual_block_map;

    void WriteOutputParameters();
};

struct CameraCameraProgram : CeresProgram {
    explicit CameraCameraProgram(CameraCameraCalibrationAppParser);

    CameraCameraProgram();

    bool PopulateProblem() override;

    void SetPresolveCameraExtrinsicsConstants();

    std::map<std::string, CameraParameterPack> camera_parameter_packs;
    std::map<std::string, std::map<unsigned long long, std::shared_ptr<BoardPoseParameterPack>>> board_pose_parameter_packs;
    CameraCamera2D3DTargetDetectionData parsed_alignment_data;
    std::map<std::string, std::map<unsigned long long, ceres::ResidualBlockId>> intrinsics_residuals_of_camera_at_time;
    std::map<std::string,
            std::map<std::string,
            std::map<unsigned long long, ceres::ResidualBlockId>>> extrinsics_residuals_of_cameras_at_time;
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

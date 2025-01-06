//
// Created by fu on 05/09/2024.
//

#include <gtboxcalibration/ceresprograms.h>
#include <gtboxcalibration/interpolation3d.h>
#include <gtboxcalibration/prismpositionresiduals.h>
#include <gtboxcalibration/argparsers.h>

#include <memory>

CameraCamera2D3DTargetDetectionData
FixCameraDetectionTimes(const CameraCamera2D3DTargetDetectionData &input_camera_detections,
                        const PrismPositionDetectionData &prism_detections) {
    const auto first_prism_stamp = prism_detections.begin()->first;
    const auto first_camera_stamp = input_camera_detections.observations.begin()->first;
    std::cout << "First prism: " << first_prism_stamp << std::endl;
    std::cout << "First camera: " << first_camera_stamp << std::endl;
    const auto camera_time_to_add = first_prism_stamp - first_camera_stamp;
    CameraCamera2D3DTargetDetectionData fixed_camera_data;
    for (const auto &stamp: input_camera_detections.unique_timestamps) {
        fixed_camera_data.unique_timestamps.insert(stamp + camera_time_to_add);
        for (const auto &[cam_name, data]: input_camera_detections.observations.at(stamp)) {
            fixed_camera_data.observations[stamp + camera_time_to_add][cam_name] = data;
        }
    }
    return fixed_camera_data;
}

CameraCameraProgram::CameraCameraProgram(CameraCameraCalibrationAppParser argparser) {
    parsed_alignment_data = FetchMulticamera2D3DDetectionData(argparser.alignment_data_path);
    board_pose_parameter_packs = PopulateBoardParameters(parsed_alignment_data);
    camera_parameter_packs = PopulateCameraParameterPacks(
            argparser.intrinsics_path, argparser.initial_guess_path);
    std::cout << "Working on " << parsed_alignment_data.unique_timestamps.size() << " time samples" << std::endl;
    std::map<std::string, std::map<unsigned long long, ceres::ResidualBlockId >> residual_block_id_of_camera_at_time3d;
    origin_camera_frame_id = camera_parameter_packs.begin()->first;
    this->PopulateProblem();
}

bool CameraCameraProgram::PopulateProblem() {
    for (auto timestamp_it = parsed_alignment_data.unique_timestamps.begin();
         timestamp_it != parsed_alignment_data.unique_timestamps.end(); ++timestamp_it) {

        const auto stamp = *timestamp_it;
        const auto next_stamp_it = std::next(timestamp_it);
        const auto &data_at_stamp = parsed_alignment_data.observations.at(stamp);

        for (auto it = data_at_stamp.begin(); it != data_at_stamp.end(); ++it) {
            std::string cam_i_name = it->first;
            const auto observation_i = it->second;
            intrinsics_residuals_of_camera_at_time[cam_i_name][stamp] =
                    addBoardPoseParameterAndCameraIntrinsicsResidualFromObservation(stamp, observation_i);
            SE3Transform se3board;

            for (auto jt = std::next(it); jt != data_at_stamp.end(); jt++) {
                std::string cam_j_name = jt->first;
                const auto observation_j = jt->second;
                problem_->getProblem().AddResidualBlock(
                        Model3DSensorAWorldModel3DSensorBWorldConsistencyError::Create(
                                observation_i.modelpoints3d,
                                observation_i.modelpoints3d.cols() * 3),
                        new ceres::HuberLoss(0.002),
                        board_pose_parameter_packs.at(cam_i_name).at(stamp)->T_sensor_board,
                        camera_parameter_packs.at(cam_i_name).T_bundle_sensor,
                        board_pose_parameter_packs.at(cam_j_name).at(stamp)->T_sensor_board,
                        camera_parameter_packs.at(cam_j_name).T_bundle_sensor);
                se3board.handleSetParameterization(problem_->getProblem(),
                                                   camera_parameter_packs.at(cam_i_name).T_bundle_sensor);
                se3board.handleSetParameterization(problem_->getProblem(),
                                                   camera_parameter_packs.at(cam_j_name).T_bundle_sensor);
            }

            if (next_stamp_it == parsed_alignment_data.unique_timestamps.end()) {
                continue;
            }
            const auto next_stamp = *next_stamp_it;
            if (next_stamp - stamp > static_cast<unsigned long long> (0.02 * 1e9)) {
                continue;
            }

            const auto &data_at_next_stamp = parsed_alignment_data.observations.at(next_stamp);
            for (const auto &jt: data_at_next_stamp) {
                std::string cam_j_name = jt.first;
                if (cam_j_name == cam_i_name) {
                    continue;
                }
                const auto observation_j = jt.second;
                problem_->getProblem().AddResidualBlock(
                        Model3DSensorAWorldModel3DSensorBWorldConsistencyError::Create(
                                observation_i.modelpoints3d,
                                observation_i.modelpoints3d.cols() * 3),
                        new ceres::HuberLoss(0.002),
                        board_pose_parameter_packs.at(cam_i_name).at(stamp)->T_sensor_board,
                        camera_parameter_packs.at(cam_i_name).T_bundle_sensor,
                        board_pose_parameter_packs.at(cam_j_name).at(next_stamp)->T_sensor_board,
                        camera_parameter_packs.at(cam_j_name).T_bundle_sensor);
                se3board.handleSetParameterization(problem_->getProblem(),
                                                   camera_parameter_packs.at(cam_i_name).T_bundle_sensor);
                se3board.handleSetParameterization(problem_->getProblem(),
                                                   camera_parameter_packs.at(cam_j_name).T_bundle_sensor);
            }
        }
    }
    return true;
}

ceres::ResidualBlockId CameraCameraProgram::addExtrinsicResidualFromObservations(const unsigned long long int stamp_i,
                                                                                 const unsigned long long int stamp_j,
                                                                                 const Observations2dModelPoints3dPointIDsPose3dSensorName &observation_i,
                                                                                 const Observations2dModelPoints3dPointIDsPose3dSensorName &observation_j,
                                                                                 const std::vector<unsigned int> &input_common_ids) {
    const auto &camera_i_name = observation_i.sensor_name;
    const auto &camera_j_name = observation_j.sensor_name;

    PinholeCamera camera_i(PinholeProjection(camera_parameter_packs.at(camera_i_name).width,
                                             camera_parameter_packs.at(camera_i_name).height),
                           camera_parameter_packs.at(camera_i_name).distortion_type);

    PinholeCamera camera_j(PinholeProjection(camera_parameter_packs.at(camera_j_name).width,
                                             camera_parameter_packs.at(camera_j_name).height),
                           camera_parameter_packs.at(camera_j_name).distortion_type);
    if (!has_initialised_extrinsics.contains(camera_i_name) or !has_initialised_extrinsics.contains(camera_j_name)) {
        if (has_initialised_extrinsics[camera_i_name]) {
            Eigen::Affine3d T_cambundle_cami = SE3Transform::toEigenAffine(
                    camera_parameter_packs.at(camera_i_name).T_bundle_sensor);
            Eigen::Affine3d T_cami_camj = observation_i.T_sensor_model * observation_j.T_sensor_model.inverse();
            Eigen::Affine3d T_cambundle_camj = T_cambundle_cami * T_cami_camj;
            SE3Transform::assignToData(T_cambundle_camj,
                                       camera_parameter_packs.at(camera_j_name).T_bundle_sensor);
            has_initialised_extrinsics[camera_j_name] = true;
            std::cout << "Initialising: " << camera_j_name << std::endl << T_cambundle_camj.matrix() << std::endl;
        }
        if (has_initialised_extrinsics[camera_j_name]) {
            Eigen::Affine3d T_cambundle_camj = SE3Transform::toEigenAffine(
                    camera_parameter_packs.at(camera_j_name).T_bundle_sensor);
            Eigen::Affine3d T_camj_cami = observation_j.T_sensor_model * observation_i.T_sensor_model.inverse();
            Eigen::Affine3d T_cambundle_cami = T_cambundle_camj * T_camj_cami;
            SE3Transform::assignToData(T_cambundle_cami,
                                       camera_parameter_packs.at(camera_i_name).T_bundle_sensor);
            has_initialised_extrinsics[camera_i_name] = true;
            std::cout << "Initialising: " << camera_i_name << std::endl << T_cambundle_cami.matrix() << std::endl;
        }
    }

    std::set<unsigned int> matching_ids(input_common_ids.begin(), input_common_ids.end());
    Eigen::Matrix3Xd model_points(3, matching_ids.size());
    model_points.setZero();
    Eigen::Matrix2Xd corner_observations_i(2, matching_ids.size());
    Eigen::Matrix2Xd corner_observations_j(2, matching_ids.size());
    int output_point_index = 0;
    for (int i = 0; i < observation_i.modelpointIDs.size(); i++) {
        unsigned int corner_id = observation_i.modelpointIDs[i];
        if (matching_ids.contains(corner_id)) {
            model_points.col(output_point_index) = observation_i.modelpoints3d.col(i);
            corner_observations_i.col(output_point_index) = observation_i.observations2d.col(i);
            output_point_index++;
        }
    }
    output_point_index = 0;
    for (int i = 0; i < observation_j.modelpointIDs.size(); i++) {
        unsigned int corner_id = observation_j.modelpointIDs[i];
        if (matching_ids.contains(corner_id)) {
            corner_observations_j.col(output_point_index) = observation_j.observations2d.col(i);
            output_point_index++;
        }
    }

    assert(camera_i_name != camera_j_name);
    auto residual_block_id = problem_->getProblem().AddResidualBlock(
            StereoCameraReprojectionError::Create(camera_i, camera_j, model_points,
                                                  corner_observations_i, corner_observations_j),
            new ceres::HuberLoss(0.5),
            board_pose_parameter_packs.at(camera_i_name).at(stamp_i)->T_sensor_board,
            camera_parameter_packs.at(camera_i_name).T_bundle_sensor,
            camera_parameter_packs.at(camera_i_name).dist_coeffs,
            camera_parameter_packs.at(camera_i_name).fxfycxcy,
            board_pose_parameter_packs.at(camera_j_name).at(stamp_j)->T_sensor_board,
            camera_parameter_packs.at(camera_j_name).T_bundle_sensor,
            camera_parameter_packs.at(camera_j_name).dist_coeffs,
            camera_parameter_packs.at(camera_j_name).fxfycxcy);
    SE3Transform transform;
    transform.handleSetParameterization(problem_->getProblem(),
                                        camera_parameter_packs.at(camera_i_name).T_bundle_sensor);
    transform.handleSetParameterization(problem_->getProblem(),
                                        camera_parameter_packs.at(camera_j_name).T_bundle_sensor);
    return residual_block_id;
}

ceres::ResidualBlockId CameraCameraProgram::addBoardPoseParameterAndCameraIntrinsicsResidualFromObservation(
        const unsigned long long stamp,
        const Observations2dModelPoints3dPointIDsPose3dSensorName &observation_i) {
    const auto &camera_name = observation_i.sensor_name;
    if (board_pose_parameter_packs.contains(camera_name)) {
        assert(!board_pose_parameter_packs.at(camera_name).contains(stamp));
    }
    board_pose_parameter_packs[camera_name][stamp] = std::make_unique<BoardPoseParameterPack>();
    SE3Transform se3board;
    se3board.assignToData(observation_i.T_sensor_model,
                          board_pose_parameter_packs[camera_name][stamp]->T_sensor_board);

    PinholeCamera camera_i(PinholeProjection(camera_parameter_packs.at(camera_name).width,
                                             camera_parameter_packs.at(camera_name).height),
                           camera_parameter_packs.at(camera_name).distortion_type);

    auto residual_block_id = problem_->getProblem().AddResidualBlock(
            TransformDistortPinholeProjectError::Create(
                    camera_i,
                    observation_i.modelpoints3d,
                    observation_i.observations2d,
                    observation_i.observations2d.cols() * 2),
            new ceres::HuberLoss(0.5),
            board_pose_parameter_packs.at(camera_name).at(stamp)->T_sensor_board,
            camera_parameter_packs.at(camera_name).dist_coeffs,
            camera_parameter_packs.at(camera_name).fxfycxcy);
    se3board.handleSetParameterization(problem_->getProblem(),
                                       board_pose_parameter_packs.at(camera_name).at(stamp)->T_sensor_board);
    return residual_block_id;
}

void CameraCameraProgram::SetPresolveCameraExtrinsicsConstants() {
    std::cout << "Holding extrinsics of " << origin_camera_frame_id << " as constant identity" << std::endl;
    Eigen::Affine3d identity = Eigen::Affine3d::Identity();
    SE3Transform::assignToData(identity, camera_parameter_packs.at(origin_camera_frame_id).T_bundle_sensor);
    problem_->getProblem().SetParameterBlockConstant(
            camera_parameter_packs.at(origin_camera_frame_id).T_bundle_sensor);

    // Solve for extrinsics, holding intrinsics and board poses constant
    for (auto &camera_pack: camera_parameter_packs) {
        problem_->getProblem().SetParameterBlockConstant(camera_pack.second.fxfycxcy);
        problem_->getProblem().SetParameterBlockConstant(camera_pack.second.dist_coeffs);
    }
    for (auto &[cam_name, board_poses]: board_pose_parameter_packs) {
        for (auto &board_pose_pack: board_poses) {
            problem_->getProblem().SetParameterBlockConstant(board_pose_pack.second->T_sensor_board);
        }
    }
}

CameraCameraProgram::CameraCameraProgram() {}


CameraPrismProgram::CameraPrismProgram(CameraPrismCalibrationAppParser argparser) {
    output_yaml_path = argparser.output_yaml_path;
    solve_time_offset = argparser.solve_time_offset;
    camera_detections = FetchMulticamera2D3DDetectionData(
            argparser.camera_corner_detections_path);
    T_bundle_cam = FetchExtrinsicsFromYamlPath(argparser.extrinsics_path);
    cameras_calibration_path = argparser.extrinsics_path;
    for (const auto &[name, transform]: T_bundle_cam) {
        std::cout << name << "\n" << transform.matrix() << std::endl;
    }
    prism_detections = LoadPrismPositions(argparser.prism_positions_path);
    camera_detections = FixCameraDetectionTimes(camera_detections, prism_detections);
    std::cout << "Using prism detections from " << prism_detections.size() << " unique stamps" << std::endl;
    cam0_name = T_bundle_cam.begin()->first;
    SE3Transform se3Transform;
    se3Transform.assignToData(Eigen::Affine3d::Identity(),
                              prism_board_in_total_station_params.T_totalstation_board);
    this->PopulateProblem();
}

bool CameraPrismProgram::PopulateProblem() {
    SE3Transform se3Transform;
    unsigned int n_camera_samples_used = 0;
    for (const auto &[camera_stamp, detections_at_stamp]: camera_detections.observations) {
        calibration_time_nsec = camera_stamp;
        for (const auto &[camera_name, detection]: detections_at_stamp) {
            n_camera_samples_used++;
            Eigen::Affine3d T_board_camera = detection.T_sensor_model.inverse();
            Eigen::Affine3d T_cam_cam0 = T_bundle_cam.at(camera_name).inverse() * T_bundle_cam.at(cam0_name);
            residual_block_map[camera_name][camera_stamp] = problem_->getProblem().AddResidualBlock(
                    PrismInCam0InBoardInTotalStationConsistencyError::Create(
                            T_cam_cam0, T_board_camera,
                            camera_stamp, prism_detections),
                    new ceres::HuberLoss(1.0),
                    prism_board_in_total_station_params.T_totalstation_board,
                    prism_board_in_total_station_params.t_cam0_prism,
                    prism_board_in_total_station_params.t_offset
            );
            se3Transform.handleSetParameterization(problem_->getProblem(),
                                                   prism_board_in_total_station_params.T_totalstation_board);
        }
    }
    std::cout << "Added " << n_camera_samples_used << " camera samples" << std::endl;
    problem_->solver_options_.minimizer_progress_to_stdout = true;
    problem_->solver_options_.max_num_iterations = 200;
    if (!solve_time_offset) {
        problem_->getProblem().SetParameterBlockConstant(prism_board_in_total_station_params.t_offset);
    }
    return true;
}

void CameraPrismProgram::WriteOutputParameters() {
    Eigen::Map<Eigen::Vector3d> t_cam0_prism(prism_board_in_total_station_params.t_cam0_prism);
    std::cout << "t_cam0_prism: " << t_cam0_prism.transpose() << std::endl;
    std::cout << "time offset: " << prism_board_in_total_station_params.t_offset[0] << std::endl;
    Eigen::Affine3d T_totalstation_board = SE3Transform::toEigenAffine(
            prism_board_in_total_station_params.T_totalstation_board);
    std::cout << "T_totalstation_board:\n" << T_totalstation_board.matrix() << std::endl;
    {
        std::vector<const double *> covariance_block = {prism_board_in_total_station_params.t_cam0_prism};
        Eigen::MatrixXd prism_position_covariance;
        problem_->ComputeAndFetchCovariance(
                covariance_block, prism_position_covariance);
        std::cout << "Prism position sigma:" << "\n" << prism_position_covariance.diagonal().array().sqrt().transpose()
                  << std::endl;
    }

    {
        std::vector<const double *> covariance_block = {prism_board_in_total_station_params.T_totalstation_board};
        Eigen::MatrixXd board_pose_covariance;
        problem_->ComputeAndFetchCovariance(
                covariance_block, board_pose_covariance);
        std::cout << "Board pose sigma:" << "\n" << board_pose_covariance.diagonal().array().sqrt().transpose()
                  << std::endl;
    }

    {
        std::vector<const double *> covariance_block = {prism_board_in_total_station_params.t_offset};
        Eigen::MatrixXd t_offset_covariance;
        problem_->ComputeAndFetchCovariance(
                covariance_block, t_offset_covariance);
        std::cout << "T-offset sigma:" << "\n" << t_offset_covariance.array().sqrt() << std::endl;
    }

    YAML::Node output_calibration = YAML::LoadFile(cameras_calibration_path);
    for (YAML::const_iterator it = output_calibration.begin(); it != output_calibration.end(); ++it) {
        std::string key = it->first.as<std::string>();
        std::string rostopic = it->second["rostopic"].as<std::string>();
        Eigen::Affine3d T_cam_cam0 = T_bundle_cam.at(rostopic).inverse() * T_bundle_cam.at(cam0_name);
        Eigen::Vector3d t_cam_prism = T_cam_cam0 * t_cam0_prism;
        // Convert t_cam_prism to a std::vector for YAML
        std::vector<double> t_cam_prism_vec = {t_cam_prism.x(), t_cam_prism.y(), t_cam_prism.z()};
        // Add t_cam_prism to the cameras_calibration node
        output_calibration[key]["t_cam_prism"] = t_cam_prism_vec;
    }


    std::ofstream fout(output_yaml_path);
    fout << "#Prism Calibration Data Time: " << GetHumanReadableTime(calibration_time_nsec) << std::endl;
    fout << output_calibration;
    fout.close();
    std::cout << "Prism calibration written to: " << output_yaml_path << std::endl;


}

CeresProgram::CeresProgram() {
    problem_ = std::make_unique<CeresProblem>();
}

bool CeresProgram::Solve() {
    ceres::Solve(problem_->solver_options_, &problem_->problem_, &problem_->summary_);
    return problem_->summary_.termination_type == ceres::TerminationType::CONVERGENCE;
}

void CeresProgram::PrintParameterAndResidualBlockStats() {
    // Print parameter blocks
    std::cout << "Parameter Blocks: " << std::endl;
    std::vector<double *> parameter_blocks;
    problem_->problem_.GetParameterBlocks(&parameter_blocks);
    for (size_t i = 0; i < parameter_blocks.size(); ++i) {
        int size = problem_->problem_.ParameterBlockSize(parameter_blocks[i]);
        std::cout << "Parameter Block " << i << ": size = " << size << std::endl;
    }

    // Print residual blocks
    std::cout << "\nResidual Blocks: " << std::endl;
    std::vector<ceres::ResidualBlockId> residual_blocks;
    problem_->problem_.GetResidualBlocks(&residual_blocks);
    int residual_size = problem_->problem_.NumResiduals();
    std::cout << "Total Residuals " << residual_size << std::endl;
}

void CeresProgram::ResetAndRepopulateProblem() {
    problem_ = std::make_unique<CeresProblem>();
    this->PopulateProblem();
}
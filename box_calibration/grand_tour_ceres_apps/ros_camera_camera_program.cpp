//
// Created by fu on 13/11/24.
//

#include "ros_camera_camera_offline_program.h"
#include <filesystem>
#include <Eigen/Core>
#include <ros/package.h>
#include <iomanip>
#include <grand_tour_camera_detection_msgs/CameraIntrinsicsExtrinsicsSigma.h>
#include <opencv2/opencv.hpp>
#include "ros_utils.h"
#include "ros_camera_camera_program.h"


ROSCameraCameraProgram::ROSCameraCameraProgram(ROSCameraCameraParser parser) {
    const auto rostopic_camera_parameter_packs = PopulateCameraParameterPacks(parser.initial_guess_path,
                                                                              parser.initial_guess_path);

    const auto frameid_mappings = LoadRostopicFrameIDMapping(parser.rostopic_frameid_mapping_path);
    for (const auto &[rostopic, params]: rostopic_camera_parameter_packs) {
        if (!frameid_mappings.contains(rostopic)) {
            ROS_ERROR_STREAM("Could not find ROSTOPIC to frameid mapping for " + rostopic
                             + "\nPlease verify the file: " + parser.rostopic_frameid_mapping_path);
            return;
        } else {
            const std::string frame_id = frameid_mappings.at(rostopic);
            frameid2rostopic_[frame_id] = rostopic;
            rostopic2frameid_[rostopic] = frame_id;
            this->camera_parameter_packs[frame_id] = params;
            ROS_INFO_STREAM(rostopic + "--->" + frame_id);
        }
    }

    if (camera_parameter_packs.empty()) {
        return;
    } else {
        const std::string &first_frameid = camera_parameter_packs.begin()->first;
        this->setOriginCameraFrame(first_frameid);
        this->resetCornerObservationVoxelMap();
        ROS_INFO_STREAM("Using " + first_frameid + " as the origin");
    }
    output_path = parser.output_path;
}

bool ROSCameraCameraProgram::addAlignmentData(ros::Time current_ros_time,
                                              const grand_tour_camera_detection_msgs::CameraDetections &camera_detections,
                                              bool force, bool block_viz) {
    const unsigned long stamp = camera_detections.header.stamp.toNSec();
    Observations2dModelPoints3dPointIDsPose3dSensorName observation = buildObservationFromRosMSG(camera_detections);
    ScopedTimer timer;
    if (!this->computeAndPopulateInitialGuessModelPose(observation)) {
        return false;
    }
    if (!force) {
        if (!this->handleAddIntrinsicsCost(stamp, observation, false)) {
            return false;
        }
        if (this->recordBoardPoseAndTrimHistory(stamp, observation, false)) {
            this->handleAddExtrinsicsCost(stamp, observation, false);
        }
    } else {
        if (!this->handleAddIntrinsicsCost(stamp, observation, true)) {
            return false;
        }
        this->recordBoardPoseAndTrimHistory(stamp, observation, true);
        this->handleAddExtrinsicsCost(stamp, observation, true);
    }

    if (!block_viz) {
        this->publishDetectionsUsed(camera_detections);
    }

    this->parsed_alignment_data.unique_timestamps.insert(stamp);
    this->parsed_alignment_data.observations[stamp][camera_detections.header.frame_id] = observation;
    return true;
}

bool ROSCameraCameraProgram::computeAndPopulateInitialGuessModelPose(
        Observations2dModelPoints3dPointIDsPose3dSensorName &observation) {
    if (!this->camera_parameter_packs.contains(observation.sensor_name)) {
        ROS_ERROR_STREAM("Received observation with unknown frame id: " + observation.sensor_name);
        return false;
    }
    return solvePnP(this->camera_parameter_packs[observation.sensor_name],
                    observation.observations2d, observation.modelpoints3d, observation.T_sensor_model);
}

bool
ROSCameraCameraProgram::recordBoardPoseAndTrimHistory(unsigned long long stamp,
                                                      Observations2dModelPoints3dPointIDsPose3dSensorName &new_observation,
                                                      bool force) {
    const std::string &new_observation_name = new_observation.sensor_name;
    board_pose_in_sensor_at_time_[new_observation_name][stamp] = new_observation.T_sensor_model;
    if (!force) {
        this->trimBoardPoseBuffer(new_observation_name);
    }
    return true;
}

void ROSCameraCameraProgram::trimBoardPoseBuffer(const std::string &new_observation_name) {
    while (board_pose_in_sensor_at_time_.at(new_observation_name).size() > 10) {
        board_pose_in_sensor_at_time_.at(new_observation_name).erase(
                board_pose_in_sensor_at_time_.at(new_observation_name).begin());
    }
}

bool ROSCameraCameraProgram::handleAddIntrinsicsCost(unsigned long long stamp,
                                                     Observations2dModelPoints3dPointIDsPose3dSensorName &new_observation,
                                                     bool force) {
    const bool added_to_observation_voxel_map =
            corner_detection2d_voxel_map_[new_observation.sensor_name].addToMapIfAnyIsBelowCapacity(
                    new_observation.observations2d, 5);
    if (added_to_observation_voxel_map) {
        const auto tentative_residual_block =
                this->addBoardPoseParameterAndCameraIntrinsicsResidualFromObservation(stamp, new_observation);
        intrinsics_residuals_of_camera_at_time[new_observation.sensor_name][stamp] = tentative_residual_block;
        return true;
    } else {
        return false;
    }
}

std::map<std::string, CameraCovariance> ROSCameraCameraProgram::computeCovariances() {
    if (ready_for_extrinsics_) {
        this->setExtrinsicParametersVariableBeforeOpt();
    }
    const auto total_in_out_edges = this->getTotalInAndOutExtrinsicEdges();
    std::map<std::string, CameraCovariance> covariances;
    std::map<std::string, bool> do_compute_extrinsics;
    std::vector<const double *> diagonal_covariance_blocks;
    for (const auto &[name, params]: camera_parameter_packs) {
        if (!intrinsics_residuals_of_camera_at_time.contains(name) or
            intrinsics_residuals_of_camera_at_time.at(name).empty()) {
            continue;
        }
        diagonal_covariance_blocks.push_back(params.fxfycxcy);
        if (!total_in_out_edges.contains(name)) {
            continue;
        }
        const bool compute_extrinsics_sigma = total_in_out_edges.at(name) > 0;
        if (compute_extrinsics_sigma) {
            diagonal_covariance_blocks.push_back(params.T_bundle_sensor);
            do_compute_extrinsics[name] = true;
        }
    }
    const auto covariance_object = problem_->ComputeSubBlockCovariance(diagonal_covariance_blocks);
    if (covariance_object == nullptr) {
        ROS_ERROR_STREAM("Failed to compute covariance");
        return covariances;
    }
    for (const auto &[name, params]: camera_parameter_packs) {
        if (!intrinsics_residuals_of_camera_at_time.contains(name) or
            intrinsics_residuals_of_camera_at_time.at(name).empty()) {
            continue;
        }
        std::vector<Eigen::MatrixXd> intrinsics_extrinsics_covariance;
        std::vector<const double *> local_params;
        local_params.push_back(params.fxfycxcy);
        if (do_compute_extrinsics.contains(name)) {
            local_params.push_back(params.T_bundle_sensor);
        }
        CameraCovariance local_covariance;
        if (problem_->FetchSubBlockCovariance(covariance_object, local_params,
                                              intrinsics_extrinsics_covariance)) {
            local_covariance.fxfycxcy_sigma =
                    intrinsics_extrinsics_covariance[0].diagonal().array().sqrt();
            if (do_compute_extrinsics.contains(name)) {
                local_covariance.rtvec_sigma =
                        intrinsics_extrinsics_covariance[1].diagonal().array().sqrt();
            }
            covariances[name] = local_covariance;
        }
    }
    return covariances;
}

bool ROSCameraCameraProgram::getReprojectionResiduals(ceres::Problem &problem,
                                                      const ceres::ResidualBlockId &residual_block,
                                                      Eigen::Matrix2Xf &residuals) const {

    // Loop over each residual block
    const ceres::CostFunction *cost_function = problem.GetCostFunctionForResidualBlock(residual_block);

    // Retrieve parameter blocks
    std::vector<double *> parameter_blocks;
    problem.GetParameterBlocksForResidualBlock(residual_block, &parameter_blocks);

    // Allocate space for residuals
    std::vector<double> block_residuals(cost_function->num_residuals());

    // Evaluate the residuals
    cost_function->Evaluate(parameter_blocks.data(), block_residuals.data(), nullptr);

    Eigen::Matrix2Xd residuals2d = Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>>(block_residuals.data(),
                                                                                        2,
                                                                                        block_residuals.size() /
                                                                                        2);

    // Conservatively resize mat1 to have the required number of columns
    residuals.conservativeResize(residuals.rows(), residuals.cols() + residuals2d.cols());

    // Append mat2 columns to mat1
    residuals.block(0, residuals.cols() - residuals2d.cols(), residuals.rows(),
                    residuals2d.cols()) = residuals2d.cast<float>();

    return true;
}

std::map<std::string, std::vector<Observations2dReprojectionResiduals>>
ROSCameraCameraProgram::getObservationsAndResiduals2D() const {
    std::map<std::string, std::vector<Observations2dReprojectionResiduals>> result;
    for (const auto &[stamp, observations_map]: parsed_alignment_data.observations) {
        for (const auto &[name, observation]: observations_map) {
            if (intrinsics_residuals_of_camera_at_time.contains(name) and
                intrinsics_residuals_of_camera_at_time.at(name).contains(stamp)) {
                Eigen::Matrix2Xf reprojection_residuals;
                this->getReprojectionResiduals(
                        problem_->getProblem(),
                        intrinsics_residuals_of_camera_at_time.at(name).at(stamp), reprojection_residuals);
                result[name].push_back({observation.observations2d, reprojection_residuals.cast<double>()});
            }
        }
    }
    return result;
};

bool ROSCameraCameraProgram::handleAddExtrinsicsCost(unsigned long long stamp,
                                                     Observations2dModelPoints3dPointIDsPose3dSensorName &new_observation,
                                                     bool force) {

    bool information_added = false;
    for (const auto &[other_sensor_name, other_sensor_pose_in_board_at_time]: board_pose_in_sensor_at_time_) {
        if (other_sensor_name == new_observation.sensor_name) {
            continue;
        }
        const auto next_stamp_in_other_frame = other_sensor_pose_in_board_at_time.lower_bound(stamp);
        if (next_stamp_in_other_frame == other_sensor_pose_in_board_at_time.end()) continue;
        auto prev_stamp_in_other_frame = next_stamp_in_other_frame;
        std::advance(prev_stamp_in_other_frame, -1);
        if (prev_stamp_in_other_frame == other_sensor_pose_in_board_at_time.begin()) continue;

        const double time_diff_forwards = (next_stamp_in_other_frame->first - stamp) * 1e-9;
        if (time_diff_forwards > association_time_tolerance_secs_) {
            continue;
        }
        const double time_diff_backwards = (stamp - prev_stamp_in_other_frame->first) * 1e-9;
        if (time_diff_backwards > association_time_tolerance_secs_) {
            continue;
        }
        const auto &T_sensor_board_tm1 = prev_stamp_in_other_frame->second;
        const auto &T_sensor_board_tp1 = next_stamp_in_other_frame->second;
        const double displacement = (T_sensor_board_tm1.inverse() * T_sensor_board_tp1).translation().norm();
        if (displacement > max_intersample_displacement_m) {
            continue;
        }

        // Is assignment
        const auto &other_observation =
                CameraCameraProgram::parsed_alignment_data.observations.at(prev_stamp_in_other_frame->first).at(
                        other_sensor_name);
        const auto common_ids = fetchIntersection(
                new_observation.modelpointIDs, other_observation.modelpointIDs);
        if (common_ids.empty()) {
            continue;
        }

        has_initialised_extrinsics[origin_camera_frame_id] = true;
//        if (!has_initialised_extrinsics.contains(new_observation.sensor_name) and
//            !has_initialised_extrinsics.contains(other_sensor_name)) {
//            ROS_DEBUG_STREAM("Not adding extrinsics sample between: " + new_observation.sensor_name + " and "
//                             + other_sensor_name);
//            continue;
//        }
        CameraCameraProgram::extrinsics_residuals_of_cameras_at_time[
                new_observation.sensor_name][
                other_sensor_name][
                stamp] = CameraCameraProgram::addExtrinsicResidualFromObservations(stamp,
                                                                                   prev_stamp_in_other_frame->first,
                                                                                   new_observation,
                                                                                   other_observation,
                                                                                   common_ids);

        this->problem_->getProblem().SetParameterBlockConstant(
                CameraCameraProgram::camera_parameter_packs[new_observation.sensor_name].T_bundle_sensor);
        this->problem_->getProblem().SetParameterBlockConstant(
                CameraCameraProgram::camera_parameter_packs[other_sensor_name].T_bundle_sensor);
        this->AddNewSensorVertexToObservationGraph(new_observation.sensor_name);
        this->AddNewSensorVertexToObservationGraph(other_sensor_name);

        for (const auto &id: common_ids) {
            const auto edge = gt_box::graph_utils::addEdge(frame_id_to_vertex_mapping_[other_sensor_name],
                                                           frame_id_to_vertex_mapping_[new_observation.sensor_name],
                                                           codetection_graph_);
        }
        information_added = true;
        camera_camera_adjacency_count[new_observation.sensor_name][other_sensor_name]++;
        camera_camera_adjacency_count[other_sensor_name][new_observation.sensor_name]++;
    }
    return information_added;
}

/**
 * @brief Adds a new sensor vertex to the observation graph if it doesn't already exist.
 *
 * This method checks if a sensor with the given name exists in the frame_id_to_vertex_mapping_ map.
 * If the sensor does not exist, it adds a new vertex to the codetection_graph_ and updates the
 * mapping between frame IDs and vertex indices. It also assigns the name to the newly created vertex
 * in the graph.
 *
 * @param name The name of the sensor to be added to the observation graph.
 */
void ROSCameraCameraProgram::AddNewSensorVertexToObservationGraph(
        const std::string &name) {
    if (!frame_id_to_vertex_mapping_.contains(name)) {

        frame_id_to_vertex_mapping_[name] = boost::add_vertex(codetection_graph_);
        vertex_to_frame_id_[frame_id_to_vertex_mapping_[name]] = name;
        boost::put(boost::vertex_name,
                   codetection_graph_,
                   frame_id_to_vertex_mapping_[name],
                   name);
    }
}

std::vector<unsigned int>
ROSCameraCameraProgram::fetchIntersection(const std::vector<unsigned int> &a,
                                          const std::vector<unsigned int> &b) const {// Create sets from the vectors (no need to sort manually)
    std::set<unsigned int> set1(a.begin(), a.end());
    std::set<unsigned int> set2(b.begin(), b.end());

    // Container for the result of the intersection
    std::vector<unsigned int> result;

    // Perform the set intersection
    std::set_intersection(set1.begin(), set1.end(),
                          set2.begin(), set2.end(),
                          std::back_inserter(result));
    return result;
}

bool ROSCameraCameraProgram::resetProblem() {
    parsed_alignment_data.unique_timestamps.clear();
    parsed_alignment_data.observations.clear();
    camera_camera_adjacency_count.clear();
    frame_id_to_vertex_mapping_.clear();
    vertex_to_frame_id_.clear();
    extrinsics_residuals_of_cameras_at_time.clear();
    intrinsics_residuals_of_camera_at_time.clear();
    board_pose_in_sensor_at_time_.clear();
    n_samples_last_solve_ = 0;
    board_pose_parameter_packs.clear();
    codetection_graph_.clear();
    this->problem_ = std::make_unique<CeresProblem>();
    this->resetCornerObservationVoxelMap();
    return true;
}

bool ROSCameraCameraProgram::rebuildProblemFromLoggedROSAlignmentData() {
    // Compute covariance of board poses
    this->resetStateFromLoggedObservations(true);
    this->filterOutOutliersFromLoggedObservations(5.0);
    this->resetStateFromLoggedObservations(false);
    return true;
}

void ROSCameraCameraProgram::filterOutOutliersFromLoggedObservations(double max_reprojection_error) {
    unsigned long n_filtered_out = 0;
    for (auto &[frame_id, data]: logged_ros_alignment_data_) {
        for (auto it = data.begin(); it != data.end();) {
            auto msg = it->second;
            Eigen::Matrix2Xf residuals;
            getReprojectionResiduals(this->problem_->getProblem(),
                                     intrinsics_residuals_of_camera_at_time.at(frame_id).at(
                                             msg.header.stamp.toNSec()),
                                     residuals);
            const double max_residual = residuals.cwiseAbs().maxCoeff();
            if (max_residual > max_reprojection_error) {
                // is outlier detection
                it = data.erase(it);  // Erase the element and move the iterator to the next one
                n_filtered_out++;
            } else {
                ++it;
            }
        }

        for (auto it = data.begin(); it != data.end();) {
            if (!CameraCameraProgram::extrinsics_residuals_of_cameras_at_time.contains(frame_id)) {
                break;
            }
            bool has_outlier = false;
            for (const auto &[other_frame, residual_block_at_time]: CameraCameraProgram::extrinsics_residuals_of_cameras_at_time.at(
                    frame_id)) {
                const auto msg = it->second;
                if (!residual_block_at_time.contains(msg.header.stamp.toNSec())) {
                    continue;
                }
                Eigen::Matrix2Xf residuals;
                getReprojectionResiduals(this->problem_->getProblem(),
                                         residual_block_at_time.at(msg.header.stamp.toNSec()),
                                         residuals);
                const double max_residual = residuals.cwiseAbs().maxCoeff();
                if (max_residual > max_reprojection_error) {
                    // is outlier detection
                    it = data.erase(it);  // Erase the element and move the iterator to the next one
                    n_filtered_out++;
                    has_outlier = true;
                    break;
                }
            }
            if (!has_outlier) {
                ++it;
            }
        }
    }
    std::cout << "Filtered out " << n_filtered_out << " outliers" << std::endl;
}

void ROSCameraCameraProgram::resetStateFromLoggedObservations(bool block_viz) {
    this->resetProblem();
    for (const auto &[name, data_at_time]: logged_ros_alignment_data_) {
        unsigned long index = 0;
        const unsigned long n = data_at_time.size();
        for (const auto &[stamp, msg]: data_at_time) {
            addAlignmentData(ros::Time(stamp / 1000000000ul, (stamp % 1000000000ul)),
                             msg, true, block_viz or (index < n-1) );
            index++;
        }
    }
    n_samples_last_solve_ = parsed_alignment_data.unique_timestamps.size();
}

bool ROSCameraCameraProgram::setOriginCameraFrame(const std::string &frame_id) {
    if (!this->camera_parameter_packs.contains(frame_id)) {
        return false;
    }
    this->origin_camera_frame_id = frame_id;
    return true;
}

std::stringstream GetTimeNowString(ros::Time time) {
    // Get the current ROS time
    ros::Time current_time = time;
    // Convert ROS time to time_t (which represents seconds since the Unix epoch)
    std::time_t raw_time = current_time.sec;
    // Convert to a tm structure for local time
    std::tm *time_info = std::localtime(&raw_time);
    // Convert ROS time to human-readable format
    std::stringstream time_stream;
    time_stream << std::put_time(time_info, "%Y-%m-%d-%H-%M-%S") << std::endl;
    return time_stream;
}

bool ROSCameraCameraProgram::writeCalibrationOutput() {
    const auto write_path = this->fetchOutputPath();
    if (write_path.string() == "") {
        return false;
    }
    std::map<std::string, CameraParameterPack> camera_parameters_by_rostopic;
    auto covariance_sigmas = this->computeCovariances();
    auto intrinsics_residuals = this->getObservationsAndResiduals2D();
    for (const auto &[frameid, params]: camera_parameter_packs) {
        camera_parameters_by_rostopic[frameid2rostopic_.at(frameid)] = params;
        covariance_sigmas[frameid2rostopic_.at(frameid)] = covariance_sigmas[frameid];
        intrinsics_residuals[frameid2rostopic_.at(frameid)] = intrinsics_residuals[frameid];
    }

    SerialiseCameraParameters(write_path.string(), camera_parameters_by_rostopic,
                              GetTimeNowString(calibration_time).str(),
                              std::make_shared<std::map<std::string, CameraCovariance>>(covariance_sigmas),
                              std::make_shared<std::map<std::string, std::vector<Observations2dReprojectionResiduals>>>(
                                      intrinsics_residuals));

    ROS_INFO_STREAM("Wrote output calibrations to: " + write_path.string());
    return true;
}

void ROSCameraCameraProgram::setIntrinsicParametersConstBeforeOpt() {
    for (auto &[name, params]: camera_parameter_packs) {
        if (!intrinsics_residuals_of_camera_at_time.contains(name)) {
            continue;
        }
        problem_->getProblem().SetParameterBlockConstant(params.fxfycxcy);
        problem_->getProblem().SetParameterBlockConstant(params.dist_coeffs);
    }
}

void ROSCameraCameraProgram::setIntrinsicParametersVariableBeforeOpt() {
    for (auto &[name, params]: camera_parameter_packs) {
        if (!intrinsics_residuals_of_camera_at_time.contains(name)) {
            continue;
        }
        problem_->getProblem().SetParameterBlockVariable(params.fxfycxcy);
        problem_->getProblem().SetParameterBlockVariable(params.dist_coeffs);
    }
}

void ROSCameraCameraProgram::setExtrinsicParametersVariableBeforeOpt() {// Free up camera transforms before solve
    for (auto &[name, params]: camera_parameter_packs) {
        if (!camera_camera_adjacency_count.contains(name)) {
            continue;
        }
        if (name == origin_camera_frame_id) {
            problem_->getProblem().SetParameterBlockConstant(params.T_bundle_sensor);
        } else {
            problem_->getProblem().SetParameterBlockVariable(params.T_bundle_sensor);
        }
    }
}

void ROSCameraCameraProgram::setExtrinsicParametersConstBeforeOpt() {
    for (auto &[name, params]: camera_parameter_packs) {
        if (!extrinsics_residuals_of_cameras_at_time.contains(name)) {
            continue;
        }
        problem_->getProblem().SetParameterBlockConstant(params.T_bundle_sensor);
    }
}

void ROSCameraCameraProgram::setBoardPoseParametersConst() {
    for (const auto &[cam_name, board_packs]: board_pose_parameter_packs) {
        for (const auto &[stamp, board_param]: board_packs) {
            problem_->getProblem().SetParameterBlockConstant(board_param->T_sensor_board);
        }
    }
}

void ROSCameraCameraProgram::setBoardPoseParametersVariable() {
    for (const auto &[cam_name, board_packs]: board_pose_parameter_packs) {
        for (const auto &[stamp, board_param]: board_packs) {
            problem_->getProblem().SetParameterBlockVariable(board_param->T_sensor_board);
        }
    }
}

fs::path ROSCameraCameraProgram::fetchOutputPath() {
    // if (output_path == "") {
        std::string calib_package_name = "box_calibration";
        std::filesystem::path calib_root_path = ros::package::getPath(calib_package_name);
        if (calib_root_path.empty()) {
            ROS_ERROR_STREAM("Failed to write output calibration");
            return "";
        }
        std::filesystem::path relative_output_path =
                "calibration/raw_calibration_output/cameras-intrinsics-extrinsics_latest.yaml";
        std::filesystem::path path = calib_root_path / relative_output_path;
        return path;
    // }
    // return output_path;
}

std::map<std::string, int> ROSCameraCameraProgram::getTotalInAndOutExtrinsicEdges() {
    std::map<std::string, int> total_edge_count;
    for (const auto &[name, other_cam_map]: camera_camera_adjacency_count) {
        for (const auto &[other_name, count]: other_cam_map) {
            total_edge_count[name] += count;
            total_edge_count[other_name] += count;
        }
    }
    for (const auto &[name, other_cam_map]: camera_camera_adjacency_count) {
        std::cout << "Camera count: " << name << " " << total_edge_count[name] << std::endl;
    }

    if (viz_) {
        std::vector<std::string> nodes;
        std::vector<std::pair<std::string, std::string>> edges;
        std::vector<int> edge_counts;
        for (const auto &[name, other_cam_map]: camera_camera_adjacency_count) {
            nodes.push_back(name);
            for (const auto &[other_name, count]: other_cam_map) {
                // Only add each undirected edge once
                if (name < other_name) {
                    edges.emplace_back(name, other_name);
                    edge_counts.push_back(count);
                }
            }
        }
        viz_->vizAdjacencyGraph("camera_adjacency_graph", nodes, edges, edge_counts);
    }

    return total_edge_count;
}

void ROSCameraCameraProgram::publishFrameTransforms() {
    if (!viz_) return;

    std::map<std::string, Eigen::Affine3d> transforms;
    for (const auto &[name, pack]: camera_parameter_packs) {
        transforms[name] = SE3Transform::toEigenAffine(pack.T_bundle_sensor);
    }
    viz_->vizFrameTransforms(transforms);
}

void ROSCameraCameraProgram::publishResiduals() {
    if (!viz_) return;

    // Intrinsics: one DepthImage per camera showing reprojection error at observed pixel positions
    const auto obs_residuals = getObservationsAndResiduals2D();
    for (const auto &[name, obs_res_vec]: obs_residuals) {
        if (!camera_parameter_packs.contains(name)) continue;
        const auto &params = camera_parameter_packs.at(name);
        std::vector<std::array<float, 2>> observations;
        std::vector<float> magnitudes;
        for (const auto &obs_res: obs_res_vec) {
            for (int i = 0; i < obs_res.observations2d.cols(); ++i) {
                observations.push_back({static_cast<float>(obs_res.observations2d(0, i)),
                                        static_cast<float>(obs_res.observations2d(1, i))});
                magnitudes.push_back(static_cast<float>(obs_res.reprojection_residuals.col(i).norm()));
            }
        }
        viz_->vizResidualMap(name + "/intrinsics_residuals", observations, magnitudes,
                             params.width, params.height);
    }

    // Extrinsics: one DepthImage per camera pair showing stereo reprojection error
    for (const auto &[frame_id, other_frame_map]: extrinsics_residuals_of_cameras_at_time) {
        if (!camera_parameter_packs.contains(frame_id)) continue;
        const auto &params = camera_parameter_packs.at(frame_id);
        for (const auto &[other_frame, stamp_map]: other_frame_map) {
            std::vector<std::array<float, 2>> observations;
            std::vector<float> magnitudes;
            for (const auto &[stamp, residual_block]: stamp_map) {
                if (!parsed_alignment_data.observations.contains(stamp)) continue;
                const auto &obs_at_stamp = parsed_alignment_data.observations.at(stamp);
                if (!obs_at_stamp.contains(frame_id)) continue;
                const auto &obs = obs_at_stamp.at(frame_id);
                Eigen::Matrix2Xf residuals;
                if (!getReprojectionResiduals(problem_->getProblem(), residual_block, residuals)) continue;
                const int n = std::min(static_cast<int>(obs.observations2d.cols()),
                                       static_cast<int>(residuals.cols()));
                for (int i = 0; i < n; ++i) {
                    observations.push_back({static_cast<float>(obs.observations2d(0, i)),
                                            static_cast<float>(obs.observations2d(1, i))});
                    magnitudes.push_back(residuals.col(i).norm());
                }
            }
            if (!observations.empty()) {
                viz_->vizResidualMap(frame_id + "/extrinsics_residuals",
                                     observations, magnitudes, params.width, params.height);
            }
        }
    }
}

std::vector<std::array<float, 2>> cornersToViz(
        const std::vector<geometry_msgs::Point>& corners) {
    std::vector<std::array<float, 2>> out;
    out.reserve(corners.size());
    for (const auto& p : corners) {
        out.push_back({static_cast<float>(p.x), static_cast<float>(p.y)});
    }
    return out;
}

void voxelMapToViz(const VoxelMap2D& voxel_map,
                   std::vector<std::array<int32_t, 2>>& coords,
                   std::vector<uint32_t>& counts) {
    coords.reserve(voxel_map.data_.size());
    counts.reserve(voxel_map.data_.size());
    for (const auto& [voxel, count] : voxel_map.data_) {
        coords.push_back({voxel.x, voxel.y});
        counts.push_back(static_cast<uint32_t>(count));
    }
}

bool ROSCameraCameraProgram::publishDetectionsUsed(
            const grand_tour_camera_detection_msgs::CameraDetections& camera_detections) {
        if (viz_) {
            const std::string& frame_id = camera_detections.header.frame_id;
            const int width = this->camera_parameter_packs.at(frame_id).width;
            const int height = this->camera_parameter_packs.at(frame_id).height;

            const auto corners = cornersToViz(camera_detections.corners2d);
            viz_->vizDetections(frame_id, corners);

            const auto it = corner_detection2d_voxel_map_.find(frame_id);
            if (it != corner_detection2d_voxel_map_.end()) {
                std::vector<std::array<int32_t, 2>> coords;
                std::vector<uint32_t> counts;
                voxelMapToViz(it->second, coords, counts);
                viz_->vizVoxelMap(frame_id + "/voxel_map", coords, counts,
                                  static_cast<float>(it->second.voxel_size_), width, height);
            }
        }
        return true;
    }

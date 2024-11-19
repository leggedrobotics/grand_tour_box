//
// Created by fu on 13/11/24.
//

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
}

bool ROSCameraCameraProgram::addAlignmentData(ros::Time current_ros_time,
                                              const grand_tour_camera_detection_msgs::CameraDetections &camera_detections,
                                              bool force) {
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
        if (this->handleStationarityRequirement(stamp, observation, false)) {
            this->handleAddExtrinsicsCost(stamp, observation, false);
        }
    } else {
        if (!this->handleAddIntrinsicsCost(stamp, observation, true)) {
            return false;
        }
        this->handleStationarityRequirement(stamp, observation, true);
        this->handleAddExtrinsicsCost(stamp, observation, true);
    }

    this->publishDetectionsUsed(camera_detections);

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
ROSCameraCameraProgram::handleStationarityRequirement(unsigned long long stamp,
                                                      Observations2dModelPoints3dPointIDsPose3dSensorName &new_observation,
                                                      bool force) {
    const std::string &new_observation_name = new_observation.sensor_name;
    board_pose_in_sensor_at_time_[new_observation_name][stamp] = new_observation.T_sensor_model;
    if (force) {
        return true;
    }

    this->manageObservationHistoryBuffer(new_observation_name);

    return true;
}

void ROSCameraCameraProgram::manageObservationHistoryBuffer(const std::string &new_observation_name) {
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
                CameraCameraProgram::addBoardPoseParameterAndCameraIntrinsicsResidualFromObservation(stamp,
                                                                                                     new_observation);
        CameraCameraProgram::intrinsics_residuals_of_camera_at_time[new_observation.sensor_name][stamp] = tentative_residual_block;
        return true;
    } else {
        return false;
    }
}

bool ROSCameraCameraProgram::getReprojectionResiduals(ceres::Problem &problem,
                                                      const ceres::ResidualBlockId &residual_block,
                                                      Eigen::Matrix2Xf &residuals) {

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
    CameraCameraProgram::parsed_alignment_data.unique_timestamps.clear();
    CameraCameraProgram::parsed_alignment_data.observations.clear();
    frame_id_to_vertex_mapping_.clear();
    vertex_to_frame_id_.clear();
    CameraCameraProgram::extrinsics_residuals_of_cameras_at_time.clear();
    CameraCameraProgram::intrinsics_residuals_of_camera_at_time.clear();
    board_pose_in_sensor_at_time_.clear();
    n_samples_last_solve_ = 0;
    CameraCameraProgram::board_pose_parameter_packs.clear();
    codetection_graph_.clear();
    this->problem_ = std::make_unique<CeresProblem>();
    this->resetCornerObservationVoxelMap();
    return true;
}

bool ROSCameraCameraProgram::rebuildProblemFromLoggedROSAlignmentData() {
    // Compute covariance of board poses
    this->resetStateFromLoggedObservations();
    this->filterOutOutliersFromLoggedObservations(3.0);
    this->resetStateFromLoggedObservations();
    return true;
}

void ROSCameraCameraProgram::filterOutOutliersFromLoggedObservations(double max_reprojection_error) {
    for (auto &[frame_id, data]: logged_ros_alignment_data_) {
        for (auto it = data.begin(); it != data.end();) {
            auto msg = it->second;
            Eigen::Matrix2Xf residuals;
            getReprojectionResiduals(CeresProgram::problem_->getProblem(),
                                     CameraCameraProgram::intrinsics_residuals_of_camera_at_time.at(frame_id).at(
                                             msg.header.stamp.toNSec()),
                                     residuals);
            const double max_residual = residuals.cwiseAbs().maxCoeff();
            if (max_residual > max_reprojection_error) {
                // is outlier detection
                it = data.erase(it);  // Erase the element and move the iterator to the next one
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
                getReprojectionResiduals(CeresProgram::problem_->getProblem(),
                                         residual_block_at_time.at(msg.header.stamp.toNSec()),
                                         residuals);
                const double max_residual = residuals.cwiseAbs().maxCoeff();
                if (max_residual > max_reprojection_error) {
                    // is outlier detection
                    it = data.erase(it);  // Erase the element and move the iterator to the next one
                    has_outlier = true;
                    break;
                }
            }
            if (!has_outlier) {
                ++it;
            }
        }
    }
}

void ROSCameraCameraProgram::resetStateFromLoggedObservations() {
    this->resetProblem();
    for (const auto &[name, data_at_time]: logged_ros_alignment_data_) {
        for (const auto &[stamp, msg]: data_at_time) {
            addAlignmentData(ros::Time::now(), msg, true);
        }
    }
    n_samples_last_solve_ = CameraCameraProgram::parsed_alignment_data.unique_timestamps.size();
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
    for (const auto &[frameid, params]: camera_parameter_packs) {
        camera_parameters_by_rostopic[frameid2rostopic_.at(frameid)] = params;
    }
    SerialiseCameraParameters(write_path.string(), camera_parameters_by_rostopic,
                              GetTimeNowString(calibration_time).str());
    ROS_INFO_STREAM("Wrote output calibrations to: " + write_path.string());
    return true;
}

void ROSCameraCameraProgram::setExtrinsicParametersVariableBeforeOpt() {// Free up camera transforms before solve
    for (auto &[name, params]: camera_parameter_packs) {
        if (!extrinsics_residuals_of_cameras_at_time.contains(name)) {
            continue;
        }
        if (name == origin_camera_frame_id) {
            problem_->getProblem().SetParameterBlockConstant(params.T_bundle_sensor);
        } else {
            problem_->getProblem().SetParameterBlockVariable(params.T_bundle_sensor);
        }
    }
}

fs::path ROSCameraCameraProgram::fetchOutputPath() {
    if (output_path == "") {
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
    }
    return output_path;
}

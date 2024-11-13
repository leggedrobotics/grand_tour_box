//
// Created by fu on 27/09/24.
//

#include "ros_programs.h"
#include "ros_utils.h"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>  // Include the conversion header
#include <grand_tour_camera_detection_msgs/CameraCameraAdjacency.h>
#include <grand_tour_camera_detection_msgs/CameraIntrinsicsExtrinsicsSigma.h>
#include <grand_tour_camera_detection_msgs/CameraIntrinsicsExtrinsics.h>
#include <grand_tour_camera_detection_msgs/CameraCameraCalibrationState.h>
#include <iomanip>  // Include for setting precision
#include <ros/package.h>
#include <filesystem>  // C++17 and later
#include <random>


OnlineCameraCameraProgram::OnlineCameraCameraProgram(OnlineCameraCameraParser parser) : loop_rate_(30.0) {
    run_id_ = getCurrentTimeFormatted() + "_camera_calibration";
    recording_service_nh_.setCallbackQueue(&recording_service_queue_);
    start_recording_calibration_data_service_ = recording_service_nh_.advertiseService(
            "camera_detection_recording_id", &OnlineCameraCameraProgram::startRecordingCalibrationDataServiceCallback,
            this);
    nh_.setCallbackQueue(&general_work_queue_);

    const auto rostopic_camera_parameter_packs = PopulateCameraParameterPacks(parser.initial_guess_path,
                                                                              parser.initial_guess_path);
    for (const auto &[topic_name, _]: rostopic_camera_parameter_packs) {
        ros::Subscriber sub = nh_.subscribe<grand_tour_camera_detection_msgs::CameraDetections>(
                topic_name + "_corner_detections",
                1,
                [topic_name, this](const grand_tour_camera_detection_msgs::CameraDetectionsConstPtr &msg) {
                    this->cameraDetectionsCallback(msg, topic_name);
                });
        subscribers_.push_back(sub);
    }
    timer_ = nh_.createTimer(ros::Duration(10.0),
                             [this](const ros::TimerEvent &msg) {
                                 this->optimizationCallback(msg, false);
                             });

    const auto frameid_mappings = LoadRostopicFrameIDMapping(parser.rostopic_frameid_mapping_path);
    for (const auto [rostopic, params]: rostopic_camera_parameter_packs) {
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

            ros::Publisher added_detection_pub = nh_.advertise<grand_tour_camera_detection_msgs::CameraDetections>(
                    rostopic + "_corner_detections_added", 100);
            added_detections_publisher_[frameid_mappings.at(rostopic)] = added_detection_pub;

            ros::Publisher processed_detection_pub = nh_.advertise<grand_tour_camera_detection_msgs::CameraDetections>(
                    rostopic + "_corner_detections_used", 100);
            processed_detections_publisher_[frameid_mappings.at(rostopic)] = processed_detection_pub;

            ros::Publisher extrinsics_detection_pub = nh_.advertise<grand_tour_camera_detection_msgs::CameraDetections>(
                    rostopic + "_corner_detections_used_extrinsics", 100);
            extrinsics_detections_publisher_[frameid_mappings.at(rostopic)] = extrinsics_detection_pub;
        }
    }
    calibration_data_collection_state_publisher_ =
            nh_.advertise<grand_tour_camera_detection_msgs::CameraCameraCalibrationState>(
                    "camera_camera_online_calibration/data_accumulation_state", 100);
    output_sigma_publisher_ = nh_.advertise<grand_tour_camera_detection_msgs::CameraIntrinsicsExtrinsicsSigma>(
            "camera_camera_online_calibration/sigma",
            10);
    adjacency_publisher_ = nh_.advertise<grand_tour_camera_detection_msgs::CameraCameraAdjacency>(
            "camera_camera_online_calibration/adjacency", 100);
    intrinsics_extrinsics_publisher_ = nh_.advertise<grand_tour_camera_detection_msgs::CameraIntrinsicsExtrinsics>(
            "camera_camera_online_calibration/intrinsics_extrinsics", 100);
    stopping_service_ = nh_.advertiseService(
            "camera_camera_online_calibration/finalize",
            &OnlineCameraCameraProgram::stopOptimizationServiceCallback, this);
    ROS_INFO_STREAM("Started stopping service ");

    if (camera_parameter_packs.empty()) {
        return;
    } else {
        const std::string &first_frameid = camera_parameter_packs.begin()->first;
        this->setOriginCameraFrame(first_frameid);
        this->resetCornerObservationVoxelMap();
        ROS_INFO_STREAM("Using " + first_frameid + " as the origin");
    }
    is_valid = true;

    recording_service_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &recording_service_queue_);
    general_work_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &general_work_queue_);

    recording_service_spinner_->start();
    general_work_spinner_->start();
}

bool OnlineCameraCameraProgram::addAlignmentData(ros::Time current_ros_time,
                                                 grand_tour_camera_detection_msgs::CameraDetections camera_detections,
                                                 bool force) {
    const unsigned long stamp = camera_detections.header.stamp.toNSec();
    Observations2dModelPoints3dPointIDsPose3dSensorName observation = buildObservationFromRosMSG(camera_detections);
    ScopedTimer timer;
    if (!this->computeAndPopulateInitialGuessModelPose(observation)) {
        ROS_ERROR_STREAM("Failed to initialize pose for observation from frame: " + observation.sensor_name);
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
        this->handleAddIntrinsicsCost(stamp, observation, true);
        this->handleStationarityRequirement(stamp, observation, true);
        this->handleAddExtrinsicsCost(stamp, observation, true);
    }
    added_detections_publisher_[camera_detections.header.frame_id].publish(camera_detections);
    this->parsed_alignment_data.unique_timestamps.insert(stamp);
    this->parsed_alignment_data.observations[stamp][camera_detections.header.frame_id] = observation;
    return true;
}

bool OnlineCameraCameraProgram::setOriginCameraFrame(const std::string &frame_id) {
    if (!this->camera_parameter_packs.contains(frame_id)) {
        return false;
    }
    this->origin_camera_frame_id = frame_id;
    return true;
}

bool OnlineCameraCameraProgram::computeAndPopulateInitialGuessModelPose(
        Observations2dModelPoints3dPointIDsPose3dSensorName &observation) {
    if (!this->camera_parameter_packs.contains(observation.sensor_name)) {
        ROS_ERROR_STREAM("Received observation with unknown frame id: " + observation.sensor_name);
        return false;
    }
    return solvePnP(this->camera_parameter_packs[observation.sensor_name],
                    observation.observations2d, observation.modelpoints3d, observation.T_sensor_model);
}

bool
OnlineCameraCameraProgram::handleStationarityRequirement(unsigned long long stamp,
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

void OnlineCameraCameraProgram::manageObservationHistoryBuffer(const std::string &new_observation_name) {
    while (board_pose_in_sensor_at_time_.at(new_observation_name).size() > 10) {
        board_pose_in_sensor_at_time_.at(new_observation_name).erase(
                board_pose_in_sensor_at_time_.at(new_observation_name).begin());
    }
}

bool OnlineCameraCameraProgram::handleAddIntrinsicsCost(unsigned long long stamp,
                                                        Observations2dModelPoints3dPointIDsPose3dSensorName &new_observation,
                                                        bool force) {
    const bool added_to_observation_voxel_map =
            corner_detection2d_voxel_map_[new_observation.sensor_name].addToMapIfAnyIsBelowCapacity(
                    new_observation.observations2d, 5);
    if (added_to_observation_voxel_map) {
        const auto tentative_residual_block =
                addBoardPoseParameterAndCameraIntrinsicsResidualFromObservation(stamp, new_observation);
        intrinsics_residuals_of_camera_at_time[new_observation.sensor_name][stamp] = tentative_residual_block;
        return true;
    } else {
        return false;
    }
}

void OnlineCameraCameraProgram::cameraDetectionsCallback(
        const grand_tour_camera_detection_msgs::CameraDetections::ConstPtr &msg,
        const std::string topic_name) {
    // Start measuring time using high-resolution clock
    {
        std::lock_guard<std::mutex> lock(ceres_problem_mutex_);
        ScopedTimer timer;
        if (this->addAlignmentData(ros::Time::now(), *msg, false)) {
            this->logged_ros_alignment_data_[msg->header.frame_id][msg->header.stamp.toNSec()] = *msg;
        } else {
            total_n_samples_rejected_++;
        }
    }
}

std::map<std::string, std::vector<unsigned long long>> getKeysToRemove(
        const std::map<std::string, std::map<unsigned long long, ceres::ResidualBlockId>>& data,
        size_t bound = 20) {

    std::map<std::string, std::vector<unsigned long long>> keys_to_remove;

    // Random number generator for shuffling
    std::random_device rd;
    std::mt19937 gen(rd());

    for (const auto& [frame_id, stamp_map] : data) {
        // Check if the inner map size exceeds the specified bound
        if (stamp_map.size() > bound) {
            std::vector<unsigned long long> keys;

            // Collect all timestamps (keys) into a vector
            for (const auto& [timestamp, _] : stamp_map) {
                keys.push_back(timestamp);
            }

            // Calculate how many keys need to be removed to stay within bound
            size_t excess_count = stamp_map.size() - bound;

            // Shuffle the keys to randomly select the excess ones
            std::shuffle(keys.begin(), keys.end(), gen);

            // Resize the keys vector to keep only the first `excess_count` keys for removal
            keys.resize(excess_count);

            // Store the keys to remove for this frame_id
            keys_to_remove[frame_id] = std::move(keys);
        }
    }

    return keys_to_remove;
}

void eraseKeys(
        std::map<std::string, std::map<unsigned long long, ceres::ResidualBlockId>>& data,
        const std::map<std::string, std::vector<unsigned long long>>& keys_to_remove) {

    for (const auto& [frame_id, keys] : keys_to_remove) {
        for (const auto& key : keys) {
            data[frame_id].erase(key);
        }
    }
}

template <typename OuterMap>
auto getTotalItemCount(const OuterMap& data) -> typename OuterMap::mapped_type::size_type {
    using InnerMap = typename OuterMap::mapped_type;
    return std::accumulate(data.begin(), data.end(), 0,
                           [](auto sum, const std::pair<typename OuterMap::key_type, InnerMap>& pair) {
                               return sum + pair.second.size();
                           }
    );
}

void OnlineCameraCameraProgram::optimizationCallback(const ros::TimerEvent &, bool force_finalise) {
    if (!do_optimize_) return;

    if (force_finalise) {
        problem_->solver_options_.max_num_iterations = 200;
        ROS_INFO_STREAM("This will be the last optimization. Unsubscribing from detection streams");
        for (auto &sub: subscribers_) {
            sub.shutdown();
        }
    } else {
        problem_->solver_options_.max_num_iterations = 5;
    }

    unsigned int n_samples_now = parsed_alignment_data.unique_timestamps.size();
    ROS_INFO_STREAM("Current n-samples: " + std::to_string(n_samples_now)
                    + ". Rejected " + std::to_string(total_n_samples_rejected_) + " samples.");
    const int n_new_samples = n_samples_now - n_samples_last_solve_;

    float current_batch_percentage_accumulated = float(n_new_samples) / float(min_new_samples_for_solve_);
    this->publishPercentageDataAccumulated(current_batch_percentage_accumulated);

    if (n_new_samples > min_new_samples_for_solve_ or force_finalise) {
        n_samples_last_solve_ = n_samples_now;
        std::lock_guard<std::mutex> lock(ceres_problem_mutex_);
        ROS_DEBUG_STREAM("Starting solve...");
        if (ready_for_extrinsics_) {
            this->setExtrinsicParametersVariableBeforeOpt();
        }
        {
            // Subsample intrinsic residuals
            int n_intrinsic_residuals = getTotalItemCount(intrinsics_residuals_of_camera_at_time);
            ROS_DEBUG_STREAM("N intrinsic residuals before downsample: " + std::to_string(n_intrinsic_residuals));
            const auto keys_to_remove = getKeysToRemove(intrinsics_residuals_of_camera_at_time,
                                                        force_finalise ? 100 : 30);
            for (const auto& [frame_id, stamp_pack] : keys_to_remove) {
                for (const auto& stamp : stamp_pack) {
                    problem_->getProblem().RemoveResidualBlock(intrinsics_residuals_of_camera_at_time.at(frame_id).at(stamp));
                }
            }
            eraseKeys(intrinsics_residuals_of_camera_at_time, keys_to_remove);
            n_intrinsic_residuals = getTotalItemCount(intrinsics_residuals_of_camera_at_time);
            ROS_DEBUG_STREAM("N intrinsic residuals after downsample: " + std::to_string(n_intrinsic_residuals));
        }
        const bool solve_succeeded = this->Solve();
        ROS_DEBUG_STREAM("Solve success: " + std::to_string(solve_succeeded));
        this->rebuildProblemFromLoggedROSAlignmentData();
        ROS_DEBUG_STREAM("Starting covariance computation...");
        ScopedTimer timer;
        const std::map<std::string, CameraCovariance> covariances = computeCovariances();
        ready_for_extrinsics_ = true;
        for (const auto &[name, covariance]: covariances) {
            if (covariance.fxfycxcy_sigma.maxCoeff() > 10.0) {
                ready_for_extrinsics_ = false;
            }
        }
        this->publishAllParamsAndSigmas(covariances);
        ROS_DEBUG("Covariance and ros publishing executed in: %f seconds", timer.elapsed().count());
        this->publishPercentageDataAccumulated(0.0f);
        {
            for (const auto &[frame_id, data]: logged_ros_alignment_data_) {
                grand_tour_camera_detection_msgs::CameraDetections output_msg;
                for (const auto &[_, msg]: data) {
                    Eigen::Matrix2Xf residuals;
                    output_msg.header = msg.header;
                    this->getReprojectionResiduals(problem_->getProblem(),
                                                   intrinsics_residuals_of_camera_at_time[frame_id][msg.header.stamp.toNSec()],
                                                   residuals);
                    ROS_ERROR_COND(msg.corners2d.size() != residuals.cols(),
                                   "Corners2d %ld does not match residuals %ld",
                                   msg.corners2d.size(), residuals.cols());

                    for (int col = 0; col < residuals.cols(); col++) {
                        // 2D corners
                        geometry_msgs::Point residual;
                        residual.x = residuals.col(col).x();
                        residual.y = residuals.col(col).y();
                        residual.z = 0.0;  // Since it's a 2D corner, the z-component is 0
                        output_msg.residuals2d.push_back(residual);
                        output_msg.corners2d.push_back(msg.corners2d[col]);
                        output_msg.modelpoint3d.push_back(msg.modelpoint3d[col]);
                        output_msg.cornerids.push_back(msg.cornerids[col]);
                    }
                }
                processed_detections_publisher_[frame_id].publish(output_msg);
            }

            if (ready_for_extrinsics_) {
                for (const auto &[cam_a, cam_b_data]: extrinsics_residuals_of_cameras_at_time) {
                    grand_tour_camera_detection_msgs::CameraDetections output_msg;
                    for (const auto &[cam_b, residual_block_at_time]: cam_b_data) {
                        for (const auto &[time, residual_block]: residual_block_at_time) {
                            output_msg.header.stamp.fromNSec(time);
                            output_msg.header.frame_id = cam_a;
                            Eigen::Matrix2Xf residuals;
                            this->getReprojectionResiduals(problem_->getProblem(),
                                                           residual_block,
                                                           residuals);
                            for (int col = 0; col < residuals.cols(); col++) {
                                // 2D corners
                                geometry_msgs::Point residual;
                                residual.x = residuals.col(col).x();
                                residual.y = residuals.col(col).y();
                                residual.z = 0.0;  // Since it's a 2D corner, the z-component is 0
                                output_msg.residuals2d.push_back(residual);
                            }
                        }
                    }
                    extrinsics_detections_publisher_[cam_a].publish(output_msg);
                }
                logEdgeCapacities();
            }
        }
    } else {
        ROS_DEBUG_STREAM("Not enough new samples received since last solve. Not optimizing");
    }

}

void OnlineCameraCameraProgram::publishPercentageDataAccumulated(float current_batch_percentage_accumulated) const {
    if (!parsed_alignment_data.unique_timestamps.empty()) {
        current_batch_percentage_accumulated =
                current_batch_percentage_accumulated < 1.0f ? current_batch_percentage_accumulated :
                1.0f;
        grand_tour_camera_detection_msgs::CameraCameraCalibrationState data_accumulation_msg;
        const unsigned long long latest_stamp = *std::prev(parsed_alignment_data.unique_timestamps.end());
        data_accumulation_msg.header.stamp.fromNSec(latest_stamp);
        data_accumulation_msg.percentage_progress.data = current_batch_percentage_accumulated;
        calibration_data_collection_state_publisher_.publish(data_accumulation_msg);
    }
}

void OnlineCameraCameraProgram::setExtrinsicParametersVariableBeforeOpt() {// Free up camera transforms before solve
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

void
OnlineCameraCameraProgram::publishAllParamsAndSigmas(const std::map<std::string, CameraCovariance> &covariances) const {
    for (const auto &[name, covariance]: covariances) {
        publishParamsAndSigmas(name, covariance.rtvec_sigma, covariance.fxfycxcy_sigma);
    }
}

std::map<std::string, CameraCovariance> OnlineCameraCameraProgram::computeCovariances() {
    if (ready_for_extrinsics_) {
        this->setExtrinsicParametersVariableBeforeOpt();
    }
    std::map<std::string, CameraCovariance> covariances;
    std::map<std::string, bool> do_compute_extrinsics;
    std::vector<const double *> diagonal_covariance_blocks;
    for (const auto &[name, params]: camera_parameter_packs) {
        if (!intrinsics_residuals_of_camera_at_time.contains(name) or
            intrinsics_residuals_of_camera_at_time.at(name).empty()) {
            continue;
        }
        diagonal_covariance_blocks.push_back(params.fxfycxcy);
        const bool compute_extrinsics_sigma = extrinsics_residuals_of_cameras_at_time.contains(name) and
                                              !extrinsics_residuals_of_cameras_at_time.at(name).empty();
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

void OnlineCameraCameraProgram::publishParamsAndSigmas(const std::string &name,
                                                       const Eigen::VectorXd &rvectvec_sigma,
                                                       const Eigen::VectorXd &fxfycxcy_sigma) const {
    grand_tour_camera_detection_msgs::CameraIntrinsicsExtrinsicsSigma output_sigma_msg;
    for (int index = 0; index < rvectvec_sigma.rows(); index++) {
        output_sigma_msg.rvectvec_sigma.push_back(rvectvec_sigma(index));
    }
    for (int index = 0; index < fxfycxcy_sigma.rows(); index++) {
        output_sigma_msg.fxfycxcy_sigma.push_back(fxfycxcy_sigma(index));
    }
    std_msgs::Header header;
    header.frame_id = name;
    const auto latest_stamp = *parsed_alignment_data.unique_timestamps.rbegin();
    header.stamp.fromNSec(latest_stamp);
    output_sigma_msg.header = header;
    output_sigma_publisher_.publish(output_sigma_msg);

    const auto &params = camera_parameter_packs.at(name);

    grand_tour_camera_detection_msgs::CameraIntrinsicsExtrinsics intrinsics_extrinsics_msg;
    intrinsics_extrinsics_msg.header = header;
    intrinsics_extrinsics_msg.fxfycxcy.resize(PinholeProjection::NUM_PARAMETERS);
    std::copy(params.fxfycxcy, params.fxfycxcy + PinholeProjection::NUM_PARAMETERS,
              intrinsics_extrinsics_msg.fxfycxcy.begin());
    intrinsics_extrinsics_msg.distcoeffs.resize(Distortion::NUM_PARAMETERS);
    std::copy(params.dist_coeffs, params.dist_coeffs + Distortion::NUM_PARAMETERS,
              intrinsics_extrinsics_msg.distcoeffs.begin());
    Eigen::Quaterniond quaternion(SE3Transform::toEigenAffine(params.T_bundle_sensor).linear());
    Eigen::Vector3d position = SE3Transform::toEigenAffine(
            params.T_bundle_sensor).translation();
    intrinsics_extrinsics_msg.T_bundle_camera.rotation.x = quaternion.x();
    intrinsics_extrinsics_msg.T_bundle_camera.rotation.y = quaternion.y();
    intrinsics_extrinsics_msg.T_bundle_camera.rotation.z = quaternion.z();
    intrinsics_extrinsics_msg.T_bundle_camera.rotation.w = quaternion.w();
    intrinsics_extrinsics_msg.T_bundle_camera.translation.x = position.x();
    intrinsics_extrinsics_msg.T_bundle_camera.translation.y = position.y();
    intrinsics_extrinsics_msg.T_bundle_camera.translation.z = position.z();
    intrinsics_extrinsics_publisher_.publish(intrinsics_extrinsics_msg);
}

bool OnlineCameraCameraProgram::getReprojectionResiduals(ceres::Problem &problem,
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

bool OnlineCameraCameraProgram::handleAddExtrinsicsCost(unsigned long long stamp,
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
                parsed_alignment_data.observations.at(prev_stamp_in_other_frame->first).at(other_sensor_name);
        const auto common_ids = fetchIntersection(
                new_observation.modelpointIDs, other_observation.modelpointIDs);
        if (common_ids.empty()) {
            continue;
        }

        extrinsics_residuals_of_cameras_at_time[
                new_observation.sensor_name][
                other_sensor_name][
                stamp] = addExtrinsicResidualFromObservations(stamp, prev_stamp_in_other_frame->first,
                                                              new_observation,
                                                              other_observation,
                                                              common_ids);

        this->problem_->getProblem().SetParameterBlockConstant(
                camera_parameter_packs[new_observation.sensor_name].T_bundle_sensor);
        this->problem_->getProblem().SetParameterBlockConstant(
                camera_parameter_packs[other_sensor_name].T_bundle_sensor);
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

void OnlineCameraCameraProgram::AddNewSensorVertexToObservationGraph(
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
OnlineCameraCameraProgram::fetchIntersection(const std::vector<unsigned int> &a,
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

void OnlineCameraCameraProgram::logEdgeCapacities() const {
    boost::graph_traits<gt_box::graph_utils::Graph>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(codetection_graph_); ei != ei_end; ++ei) {
        unsigned int capacity = boost::get(boost::edge_capacity, codetection_graph_, *ei);
        const auto a = source(*ei, codetection_graph_);
        const auto b = target(*ei, codetection_graph_);
        const std::string name_a = vertex_to_frame_id_.at(a);
        const std::string name_b = vertex_to_frame_id_.at(b);
        grand_tour_camera_detection_msgs::CameraCameraAdjacency adjacency_msg;
        // Get an iterator to the last element
        auto latest_data_timestamp = --parsed_alignment_data.unique_timestamps.end();
        const auto timestamp = ros::Time().fromNSec(*latest_data_timestamp);
        adjacency_msg.header.stamp = timestamp;
        adjacency_msg.camera_a.data = name_a;
        adjacency_msg.camera_b.data = name_b;
        adjacency_msg.capacity.data = capacity;
        adjacency_publisher_.publish(adjacency_msg);
    }
}

bool OnlineCameraCameraProgram::resetProblem() {
    parsed_alignment_data.unique_timestamps.clear();
    parsed_alignment_data.observations.clear();
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

bool OnlineCameraCameraProgram::rebuildProblemFromLoggedROSAlignmentData() {
    // Compute covariance of board poses
    this->resetStateFromLoggedObservations();
    this->filterOutOutliersFromLoggedObservations(3.0);
    this->resetStateFromLoggedObservations();
    return true;
}

void OnlineCameraCameraProgram::filterOutOutliersFromLoggedObservations(double max_reprojection_error) {
    for (auto &[frame_id, data]: logged_ros_alignment_data_) {
        for (auto it = data.begin(); it != data.end();) {
            auto msg = it->second;
            Eigen::Matrix2Xf residuals;
            getReprojectionResiduals(problem_->getProblem(),
                                     intrinsics_residuals_of_camera_at_time.at(frame_id).at(
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
            if (!extrinsics_residuals_of_cameras_at_time.contains(frame_id)) {
                break;
            }
            bool has_outlier = false;
            for (const auto &[other_frame, residual_block_at_time]: extrinsics_residuals_of_cameras_at_time.at(
                    frame_id)) {
                const auto msg = it->second;
                if (!residual_block_at_time.contains(msg.header.stamp.toNSec())) {
                    continue;
                }
                Eigen::Matrix2Xf residuals;
                getReprojectionResiduals(problem_->getProblem(),
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

void OnlineCameraCameraProgram::resetStateFromLoggedObservations() {
    this->resetProblem();
    for (const auto &[name, data_at_time]: logged_ros_alignment_data_) {
        for (const auto &[stamp, msg]: data_at_time) {
            addAlignmentData(ros::Time::now(), msg, true);
        }
    }
    n_samples_last_solve_ = parsed_alignment_data.unique_timestamps.size();
}

std::stringstream GetTimeNowString() {
    // Get the current ROS time
    ros::Time current_time = ros::Time::now();
    // Convert ROS time to time_t (which represents seconds since the Unix epoch)
    std::time_t raw_time = current_time.sec;
    // Convert to a tm structure for local time
    std::tm *time_info = std::localtime(&raw_time);
    // Convert ROS time to human-readable format
    std::stringstream time_stream;
    time_stream << "Generated with data of ROS time: ";
    time_stream << std::put_time(time_info, "%Y-%m-%d %H:%M:%S");
    // Add nanoseconds for precision
    time_stream << "." << std::setw(9) << std::setfill('0') << current_time.nsec << std::endl;
    return time_stream;
}

bool OnlineCameraCameraProgram::stopOptimizationServiceCallback(
        grand_tour_camera_detection_msgs::StopOptimizingService::Request &req,
        grand_tour_camera_detection_msgs::StopOptimizingService::Response &res) {
    ros::TimerEvent event_msg;
    ROS_INFO_STREAM("Performing last solve");
    this->optimizationCallback(event_msg, true);
    ROS_INFO_STREAM("Final solve done");
    do_optimize_ = false;
    res.successfully_stopped = true;

    std::string calib_package_name = "box_calibration";
    std::filesystem::path calib_root_path = ros::package::getPath(calib_package_name);
    if (calib_root_path.empty()) {
        ROS_ERROR_STREAM("Failed to write output calibration");
        return true;
    }
    std::filesystem::path relative_output_path =
            "calibration/raw_calibration_output/cameras-intrinsics-extrinsics_latest.yaml";
    std::filesystem::path output_path = calib_root_path / relative_output_path;
    std::map<std::string, CameraParameterPack> camera_parameters_by_rostopic;
    for (const auto &[frameid, params]: camera_parameter_packs) {
        camera_parameters_by_rostopic[frameid2rostopic_.at(frameid)] = params;
    }
    SerialiseCameraParameters(output_path.string(), camera_parameters_by_rostopic,
                              GetTimeNowString().str());
    ROS_INFO_STREAM("Wrote output calibrations to: " + output_path.string());
    start_recording_calibration_data_service_.shutdown();
    ROS_INFO_STREAM("Stopped recording calibration data");
    return true;
}

bool OnlineCameraCameraProgram::startRecordingCalibrationDataServiceCallback(
        grand_tour_camera_detection_msgs::StartRecordingCalibrationDataService::Request &req,
        grand_tour_camera_detection_msgs::StartRecordingCalibrationDataService::Response &res) {
    res.recording_id.data = run_id_;
    return true;
}




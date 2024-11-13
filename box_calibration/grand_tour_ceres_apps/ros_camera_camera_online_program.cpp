//
// Created by fu on 27/09/24.
//

#include "ros_camera_camera_online_program.h"
#include "ros_utils.h"
#include "ros_camera_camera_program.h"
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


ROSCameraCameraOnlineProgram::ROSCameraCameraOnlineProgram(ROSCameraCameraParser parser) : ROSCameraCameraProgram(
        parser), loop_rate_(30.0) {

    const auto rostopic_camera_parameter_packs = PopulateCameraParameterPacks(parser.initial_guess_path,
                                                                              parser.initial_guess_path);

    const auto frameid_mappings = LoadRostopicFrameIDMapping(parser.rostopic_frameid_mapping_path);

    run_id_ = getCurrentTimeFormatted();
    recording_service_nh_.setCallbackQueue(&recording_service_queue_);
    start_recording_calibration_data_service_ = recording_service_nh_.advertiseService(
            "camera_detection_recording_id",
            &ROSCameraCameraOnlineProgram::startRecordingCalibrationDataServiceCallback,
            this);
    nh_.setCallbackQueue(&general_work_queue_);

    for (const auto [rostopic, params]: rostopic_camera_parameter_packs) {
        if (!frameid_mappings.contains(rostopic)) {
            ROS_ERROR_STREAM("Could not find ROSTOPIC to frameid mapping for " + rostopic
                             + "\nPlease verify the file: " + parser.rostopic_frameid_mapping_path);
            return;
        } else {
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

    for (const auto &[topic_name, _]: rostopic_camera_parameter_packs) {
        ros::Subscriber sub = nh_.subscribe<grand_tour_camera_detection_msgs::CameraDetections>(
                topic_name + detection_suffix,
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
            &ROSCameraCameraOnlineProgram::stopOptimizationServiceCallback, this);
    ROS_INFO_STREAM("Started stopping service ");

    recording_service_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &recording_service_queue_);
    general_work_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &general_work_queue_);

    recording_service_spinner_->start();
    general_work_spinner_->start();
    is_valid = true;
}

void ROSCameraCameraOnlineProgram::cameraDetectionsCallback(
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
        const std::map<std::string, std::map<unsigned long long, ceres::ResidualBlockId>> &data,
        size_t bound = 20) {

    std::map<std::string, std::vector<unsigned long long>> keys_to_remove;

    // Random number generator for shuffling
    std::random_device rd;
    std::mt19937 gen(rd());

    for (const auto &[frame_id, stamp_map]: data) {
        // Check if the inner map size exceeds the specified bound
        if (stamp_map.size() > bound) {
            std::vector<unsigned long long> keys;

            // Collect all timestamps (keys) into a vector
            for (const auto &[timestamp, _]: stamp_map) {
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
        std::map<std::string, std::map<unsigned long long, ceres::ResidualBlockId>> &data,
        const std::map<std::string, std::vector<unsigned long long>> &keys_to_remove) {

    for (const auto &[frame_id, keys]: keys_to_remove) {
        for (const auto &key: keys) {
            data[frame_id].erase(key);
        }
    }
}

template<typename OuterMap>
auto getTotalItemCount(const OuterMap &data) -> typename OuterMap::mapped_type::size_type {
    using InnerMap = typename OuterMap::mapped_type;
    return std::accumulate(data.begin(), data.end(), 0,
                           [](auto sum, const std::pair<typename OuterMap::key_type, InnerMap> &pair) {
                               return sum + pair.second.size();
                           }
    );
}

void ROSCameraCameraOnlineProgram::optimizationCallback(const ros::TimerEvent &, bool force_finalise) {
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
            for (const auto &[frame_id, stamp_pack]: keys_to_remove) {
                for (const auto &stamp: stamp_pack) {
                    problem_->getProblem().RemoveResidualBlock(
                            intrinsics_residuals_of_camera_at_time.at(frame_id).at(stamp));
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

void ROSCameraCameraOnlineProgram::publishPercentageDataAccumulated(float current_batch_percentage_accumulated) const {
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

void
ROSCameraCameraOnlineProgram::publishAllParamsAndSigmas(
        const std::map<std::string, CameraCovariance> &covariances) const {
    for (const auto &[name, covariance]: covariances) {
        publishParamsAndSigmas(name, covariance.rtvec_sigma, covariance.fxfycxcy_sigma);
    }
}

std::map<std::string, CameraCovariance> ROSCameraCameraOnlineProgram::computeCovariances() {
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

void ROSCameraCameraOnlineProgram::publishParamsAndSigmas(const std::string &name,
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

void ROSCameraCameraOnlineProgram::logEdgeCapacities() const {
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

bool ROSCameraCameraOnlineProgram::stopOptimizationServiceCallback(
        grand_tour_camera_detection_msgs::StopOptimizingService::Request &req,
        grand_tour_camera_detection_msgs::StopOptimizingService::Response &res) {
    ros::TimerEvent event_msg;
    ROS_INFO_STREAM("Performing last solve");
    this->optimizationCallback(event_msg, true);
    ROS_INFO_STREAM("Final solve done");
    do_optimize_ = false;
    res.successfully_stopped = true;
    this->writeCalibrationOutput();
    start_recording_calibration_data_service_.shutdown();
    ROS_INFO_STREAM("Stopped recording calibration data");
    return true;
}

bool ROSCameraCameraOnlineProgram::startRecordingCalibrationDataServiceCallback(
        grand_tour_camera_detection_msgs::StartRecordingCalibrationDataService::Request &req,
        grand_tour_camera_detection_msgs::StartRecordingCalibrationDataService::Response &res) {
    res.recording_id.data = run_id_;
    return true;
}

bool ROSCameraCameraOnlineProgram::publishDetectionsUsed(
        const grand_tour_camera_detection_msgs::CameraDetections &camera_detections) {
    added_detections_publisher_[camera_detections.header.frame_id].publish(camera_detections);
    return true;
}




//
// Created by fu on 27/09/24.
//

#ifndef GRAND_TOUR_CERES_APPS_ROS_PROGRAMS_H
#define GRAND_TOUR_CERES_APPS_ROS_PROGRAMS_H

#include <gtboxcalibration/ceresprograms.h>
#include <ros/ros.h>
#include <grand_tour_camera_detection_msgs/CameraDetections.h>
#include <grand_tour_camera_detection_msgs/StopOptimizingService.h>
#include <grand_tour_camera_detection_msgs/StartRecordingCalibrationDataService.h>
#include "ros_parsers.h"
#include "gtboxcalibration/voxel_2d.h"
#include <gtboxcalibration/detectiongraphutils.h>
#include <ros/callback_queue.h>

struct CameraCovariance {
    Eigen::VectorXd rtvec_sigma;
    Eigen::VectorXd fxfycxcy_sigma;
};

struct OnlineCameraCameraProgram : public CameraCameraProgram {

    bool is_valid = false;

    OnlineCameraCameraProgram(OnlineCameraCameraParser);
    ~OnlineCameraCameraProgram() {
        recording_service_spinner_->stop();
        general_work_spinner_->stop();
    }

    void run() {
        while (ros::ok()) {
            // Sleep to maintain the loop rate
            loop_rate_.sleep();
        }
    }

private:
    // Timer callback (runs every 5 seconds)
    void optimizationCallback(const ros::TimerEvent &, bool force_finalise);

    void cameraDetectionsCallback(const grand_tour_camera_detection_msgs::CameraDetections::ConstPtr &msg,
                                  const std::string topic_name);

    bool stopOptimizationServiceCallback(grand_tour_camera_detection_msgs::StopOptimizingService::Request &req,
                                         grand_tour_camera_detection_msgs::StopOptimizingService::Response &res);

    bool startRecordingCalibrationDataServiceCallback(
            grand_tour_camera_detection_msgs::StartRecordingCalibrationDataService::Request &req,
            grand_tour_camera_detection_msgs::StartRecordingCalibrationDataService::Response &res);


    bool addAlignmentData(ros::Time timestamp, grand_tour_camera_detection_msgs::CameraDetections camera_detections,
                          bool force);

    bool setOriginCameraFrame(const std::string &camera_name);

    bool computeAndPopulateInitialGuessModelPose(Observations2dModelPoints3dPointIDsPose3dSensorName &observation);

    bool
    handleStationarityRequirement(unsigned long long stamp,
                                  Observations2dModelPoints3dPointIDsPose3dSensorName &new_observation,
                                  bool force);

    bool handleAddIntrinsicsCost(unsigned long long stamp,
                                 Observations2dModelPoints3dPointIDsPose3dSensorName &new_observation,
                                 bool force);

    bool handleAddExtrinsicsCost(unsigned long long stamp,
                                 Observations2dModelPoints3dPointIDsPose3dSensorName &new_observation,
                                 bool force);

    bool resetProblem();

    bool rebuildProblemFromLoggedROSAlignmentData();

    bool getReprojectionResiduals(ceres::Problem &problem,
                                  const ceres::ResidualBlockId &residual_block,
                                  Eigen::Matrix2Xf &residuals);

    void resetCornerObservationVoxelMap() {
        for (const auto &name: camera_parameter_packs) {
            corner_detection2d_voxel_map_[name.first] = VoxelMap2D(corner_detection2d_voxel_size_);
        }
    }

    // Function to log the current edge capacities
    void logEdgeCapacities() const;

    std::vector<unsigned int> fetchIntersection(const std::vector<unsigned int> &a,
                                                const std::vector<unsigned int> &b) const;

    ros::NodeHandle nh_;
    ros::NodeHandle recording_service_nh_;      // NodeHandle for service with dedicated queue

    ros::CallbackQueue recording_service_queue_;
    ros::CallbackQueue general_work_queue_;

    std::unique_ptr<ros::AsyncSpinner> recording_service_spinner_;
    std::unique_ptr<ros::AsyncSpinner> general_work_spinner_;

    ros::Rate loop_rate_;

    ros::ServiceServer stopping_service_, start_recording_calibration_data_service_;
    std::string run_id_;
    std::vector<ros::Subscriber> subscribers_;
    std::map<std::string, std::string> frameid2rostopic_, rostopic2frameid_;
    std::map<std::string, ros::Publisher> added_detections_publisher_;
    std::map<std::string, ros::Publisher> processed_detections_publisher_;
    std::map<std::string, ros::Publisher> extrinsics_detections_publisher_;
    double corner_detection2d_voxel_size_ = 8;
    std::map<std::string, VoxelMap2D> corner_detection2d_voxel_map_;
    ros::Publisher output_sigma_publisher_, intrinsics_extrinsics_publisher_, adjacency_publisher_,
            calibration_data_collection_state_publisher_;
    ros::Timer timer_;
    unsigned int total_n_samples_rejected_ = 0;
    unsigned int n_samples_last_solve_ = 0;
    unsigned int min_new_samples_for_solve_ = 300;
    std::mutex ceres_problem_mutex_;

    std::map<std::string, std::map<unsigned long long, Eigen::Affine3d>> board_pose_in_sensor_at_time_;
    double max_intersample_displacement_m = 1e-4;
    double association_time_tolerance_secs_ = 0.5;
    std::map<std::string, std::map<
            unsigned long long, grand_tour_camera_detection_msgs::CameraDetections>> logged_ros_alignment_data_;
    gt_box::graph_utils::Graph codetection_graph_;
    std::map<std::string, unsigned long> frame_id_to_vertex_mapping_;
    std::map<unsigned long, std::string> vertex_to_frame_id_;
    bool do_optimize_ = true;
    bool ready_for_extrinsics_ = false;

    void resetStateFromLoggedObservations();

    void filterOutOutliersFromLoggedObservations(double max_reprojection_error);

    void manageObservationHistoryBuffer(const std::string &new_observation_name);

    void AddNewSensorVertexToObservationGraph(const std::string &name);

    void setExtrinsicParametersVariableBeforeOpt();

    void publishParamsAndSigmas(const std::string &name,
                                const Eigen::VectorXd &rvectvec_sigma, const Eigen::VectorXd &fxfycxcy_sigma) const;

    std::map<std::string, CameraCovariance> computeCovariances();

    void publishAllParamsAndSigmas(const std::map<std::string, CameraCovariance> &covariances) const;

    void publishPercentageDataAccumulated(float current_batch_percentage_accumulated) const;
};


#endif //GRAND_TOUR_CERES_APPS_ROS_PROGRAMS_H

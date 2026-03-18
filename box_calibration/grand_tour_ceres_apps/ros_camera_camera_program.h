//
// Created by fu on 13/11/24.
//

#ifndef GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_PROGRAM_H
#define GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_PROGRAM_H


#include <sensor_msgs/Image.h>
#include <rosbag/view.h>
#include <rosbag/bag.h>
#include <ros/callback_queue.h>
#include <gtboxcalibration/detectiongraphutils.h>
#include "gtboxcalibration/voxel_2d.h"
#include "ros_camera_camera_parser.h"
#include "ros_camera_camera_program.h"
#include <grand_tour_camera_detection_msgs/StartRecordingCalibrationDataService.h>
#include <grand_tour_camera_detection_msgs/StopOptimizingService.h>
#include <grand_tour_camera_detection_msgs/CameraDetections.h>
#include <ros/ros.h>
#include <gtboxcalibration/ceresprograms.h>
#include "camera_camera_viz_interface.h"


class ROSCameraCameraProgram : public CameraCameraProgram {

public:
    explicit ROSCameraCameraProgram(ROSCameraCameraParser);

    bool isValid() const {
        return is_valid;
    }

    void setViz(std::unique_ptr<CameraCameraVizInterface> viz) {
        viz_ = std::move(viz);
    }

private:
    std::filesystem::path fetchOutputPath();
    bool computeAndPopulateInitialGuessModelPose(Observations2dModelPoints3dPointIDsPose3dSensorName &observation);

    bool
    recordBoardPoseAndTrimHistory(unsigned long long stamp,
                                  Observations2dModelPoints3dPointIDsPose3dSensorName &new_observation,
                                  bool force);

    void trimBoardPoseBuffer(const std::string &new_observation_name);

    bool handleAddIntrinsicsCost(unsigned long long stamp,
                                 Observations2dModelPoints3dPointIDsPose3dSensorName &new_observation,
                                 bool force);

    bool handleAddExtrinsicsCost(unsigned long long stamp,
                                 Observations2dModelPoints3dPointIDsPose3dSensorName &new_observation,
                                 bool force);

    void AddNewSensorVertexToObservationGraph(const std::string &name);

    std::vector<unsigned int> fetchIntersection(const std::vector<unsigned int> &a,
                                                const std::vector<unsigned int> &b) const;

    bool resetProblem();

    void filterOutOutliersFromLoggedObservations(double max_reprojection_error);

    void resetStateFromLoggedObservations(bool block_viz);

    fs::path fetchOutputPath();

protected:
    bool addAlignmentData(ros::Time timestamp,
                          const grand_tour_camera_detection_msgs::CameraDetections &camera_detections,
                          bool force, bool block_viz);

    std::map<std::string, CameraCovariance> computeCovariances();

    void setBoardPoseParametersConst();

    void setBoardPoseParametersVariable();

    virtual bool publishDetectionsUsed(const grand_tour_camera_detection_msgs::CameraDetections &camera_detections) = 0;

    bool setOriginCameraFrame(const std::string &camera_name);

    bool getReprojectionResiduals(ceres::Problem &problem,
                                  const ceres::ResidualBlockId &residual_block,
                                  Eigen::Matrix2Xf &residuals) const;

    std::map<std::string, std::vector<Observations2dReprojectionResiduals>>
            getObservationsAndResiduals2D() const;


    bool rebuildProblemFromLoggedROSAlignmentData();

    void resetCornerObservationVoxelMap() {
        for (const auto &name: CameraCameraProgram::camera_parameter_packs) {
            corner_detection2d_voxel_map_[name.first] = VoxelMap2D(corner_detection2d_voxel_size_);
        }
    }

    std::unique_ptr<CameraCameraVizInterface> viz_;

    const std::string detection_suffix = "_corner_detections";

    double corner_detection2d_voxel_size_ = 8;
    std::map<std::string, VoxelMap2D> corner_detection2d_voxel_map_;
    std::map<std::string, std::map<unsigned long long, Eigen::Affine3d>> board_pose_in_sensor_at_time_;
    double max_intersample_displacement_m = 1e-4;
    double association_time_tolerance_secs_ = 0.5;

    std::string output_path;

    std::map<std::string, unsigned long> frame_id_to_vertex_mapping_;

    std::map<std::string, std::map<
            unsigned long long, grand_tour_camera_detection_msgs::CameraDetections>> logged_ros_alignment_data_;
    unsigned int n_samples_last_solve_ = 0;

    std::map<unsigned long, std::string> vertex_to_frame_id_;
    std::map<std::string, std::string> frameid2rostopic_;
    std::map<std::string, std::string> rostopic2frameid_;
    gt_box::graph_utils::Graph codetection_graph_;
    bool is_valid = false;
    ros::Time calibration_time;
    bool ready_for_extrinsics_ = false;

    bool writeCalibrationOutput();

    void setIntrinsicParametersConstBeforeOpt();

    void setIntrinsicParametersVariableBeforeOpt();

    void setExtrinsicParametersConstBeforeOpt();

    void setExtrinsicParametersVariableBeforeOpt();

    std::map<std::string, int> getTotalInAndOutExtrinsicEdges();

    void publishResiduals();
};


#endif //GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_PROGRAM_H

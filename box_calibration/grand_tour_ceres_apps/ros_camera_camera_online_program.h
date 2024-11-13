//
// Created by fu on 27/09/24.
//

#ifndef GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_ONLINE_PROGRAM_H
#define GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_ONLINE_PROGRAM_H

#include <gtboxcalibration/ceresprograms.h>
#include <ros/ros.h>
#include <grand_tour_camera_detection_msgs/CameraDetections.h>
#include <grand_tour_camera_detection_msgs/StopOptimizingService.h>
#include <grand_tour_camera_detection_msgs/StartRecordingCalibrationDataService.h>
#include "ros_camera_camera_parser.h"
#include "gtboxcalibration/voxel_2d.h"
#include "ros_camera_camera_program.h"
#include <ros/callback_queue.h>


struct ROSCameraCameraOnlineProgram : public ROSCameraCameraProgram {

    ROSCameraCameraOnlineProgram(ROSCameraCameraParser);

    ~ROSCameraCameraOnlineProgram() {
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

protected:
    bool publishDetectionsUsed(const grand_tour_camera_detection_msgs::CameraDetections &camera_detections) override;

private:

    bool stopOptimizationServiceCallback(grand_tour_camera_detection_msgs::StopOptimizingService::Request &req,
                                         grand_tour_camera_detection_msgs::StopOptimizingService::Response &res);

    bool startRecordingCalibrationDataServiceCallback(
            grand_tour_camera_detection_msgs::StartRecordingCalibrationDataService::Request &req,
            grand_tour_camera_detection_msgs::StartRecordingCalibrationDataService::Response &res);


    void publishParamsAndSigmas(const std::string &name,
                                const Eigen::VectorXd &rvectvec_sigma, const Eigen::VectorXd &fxfycxcy_sigma) const;

    std::map<std::string, CameraCovariance> computeCovariances();

    void publishAllParamsAndSigmas(const std::map<std::string, CameraCovariance> &covariances) const;

    void publishPercentageDataAccumulated(float current_batch_percentage_accumulated) const;

    // Function to log the current edge capacities
    void logEdgeCapacities() const;

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
    std::map<std::string, ros::Publisher> processed_detections_publisher_;
    std::map<std::string, ros::Publisher> extrinsics_detections_publisher_;
    ros::Publisher output_sigma_publisher_, intrinsics_extrinsics_publisher_, adjacency_publisher_,
            calibration_data_collection_state_publisher_;
    std::map<std::string, ros::Publisher> added_detections_publisher_;

    ros::Timer timer_;
    unsigned int total_n_samples_rejected_ = 0;
    unsigned int min_new_samples_for_solve_ = 300;
    std::mutex ceres_problem_mutex_;

    bool do_optimize_ = true;
    bool ready_for_extrinsics_ = false;

};


#endif //GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_ONLINE_PROGRAM_H

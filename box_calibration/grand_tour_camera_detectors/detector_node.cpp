//
// Created by fu on 24/09/24.
//

#include "detector_node.h"
#include "aprilgridasl.h"
#include "checkerboardasl.h"

#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <filesystem>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <grand_tour_camera_detection_msgs/CameraDetections.h>
#include <grand_tour_camera_detection_msgs/StartRecordingCalibrationDataService.h>


namespace fs = std::filesystem;

void createDirectoryIfNotExists(const fs::path &filePath) {
    // Extract the parent directory from the given file path
    fs::path directory = filePath.parent_path();

    // Check if the directory already exists
    if (!fs::exists(directory)) {
        // Create the directory
        if (fs::create_directories(directory)) {
            ROS_INFO_STREAM("Directory created: " + directory.string());
        } else {
            ROS_ERROR_STREAM("Failed to create directory: " + directory.string());
        }
    } else {
        ROS_WARN_STREAM("Directory already exists: " + directory.string());
    }
}


CameraDetectorNode::CameraDetectorNode(ros::NodeHandle &nh) : nh_(nh) {
    // Use a private NodeHandle to get private parameters
    ros::NodeHandle private_nh("~");

    double show_stats_every_n_sec;
    bool use_april_grid;

    // Checkerboard specific parameters
    int rows, cols;
    double row_spacing, column_spacing;

    // Fetch parameters from the private parameter server
    private_nh.param<bool>("show_extraction_video", show_extraction_video_, false);
    private_nh.param<bool>("use_april_grid", use_april_grid, true);
    private_nh.param<double>("show_stats_every_n_sec", show_stats_every_n_sec, 30);
    private_nh.param<std::string>("image_topic", image_topic_, "/image");
    private_nh.param<std::string>("output_folder", output_root_folder_, "/data/");
    private_nh.param<std::string>("output_suffix", output_suffix_, "_corner_detections");

    if (use_april_grid) {
        private_nh.param<int>("grid_size_x", grid_size_x_, 6);
        private_nh.param<int>("grid_size_y", grid_size_y_, 6);
        private_nh.param<double>("tag_size", tag_size_, 0.083);
        private_nh.param<double>("tag_spacing", tag_spacing_, 0.3);
    } else {
        // Fetch checkerboard-specific parameters
        private_nh.param<int>("rows", rows, 7);
        private_nh.param<int>("cols", cols, 8);
        private_nh.param<double>("row_spacing", row_spacing, 0.08);
        private_nh.param<double>("column_spacing", column_spacing, 0.08);
    }

    // Choose between Aprilgrid or Checkerboard based on use_april_grid
    if (use_april_grid) {
        ROS_INFO_STREAM("USING APRILGRID");
        auto options = cameras::GridCalibrationTargetAprilgrid::AprilgridOptions();
        options.showExtractionVideo = show_extraction_video_;

        calibration_target_model_ = std::make_unique<cameras::GridCalibrationTargetAprilgrid>(
                grid_size_x_, grid_size_y_, tag_size_, tag_spacing_, options);
    } else {
        ROS_INFO_STREAM("USING CHECKERBOARD");
        auto options = cameras::GridCalibrationTargetCheckerboard::CheckerboardOptions();
        options.showExtractionVideo = show_extraction_video_;

        calibration_target_model_ = std::make_unique<cameras::GridCalibrationTargetCheckerboard>(
                rows, cols, row_spacing, column_spacing, options
        );
    }

    detection_topic_ = image_topic_ + output_suffix_;
    image_sub_ = nh_.subscribe(image_topic_, 1, &CameraDetectorNode::imageCallback, this);
    // Initialize publisher for AprilGrid detections
    detections_pub_ = nh_.advertise<grand_tour_camera_detection_msgs::CameraDetections>(detection_topic_, 1);
    recording_id_service_name_ = "camera_detection_recording_id";
    start_recording_service_client_ =
            nh_.serviceClient<grand_tour_camera_detection_msgs::StartRecordingCalibrationDataService>(
                    recording_id_service_name_);

    // Set up a periodic timer for logging statistics (every 20 seconds, for example)
    log_timer_ = nh_.createTimer(ros::Duration(show_stats_every_n_sec), &CameraDetectorNode::logStatistics, this);
    recording_id_service_timer_ = nh_.createTimer(ros::Duration(5.0),
                                                  &CameraDetectorNode::queryRecordingID, this);
}

void CameraDetectorNode::logStatistics(const ros::TimerEvent &event) {
    // Log the statistics
    ROS_INFO("Node [%s]: Images received: %d, Images with detections: %d",
             ros::this_node::getName().c_str(), images_received_, images_with_detections_);
}

bool CameraDetectorNode::openNewBag(std::string recording_id) {
    recording_id += "_calibration";
    std::string topic_name_as_path = image_topic_;
    std::replace(topic_name_as_path.begin(), topic_name_as_path.end(), '/', '_');
    fs::path full_bag_path = fs::path(output_root_folder_) / recording_id / (
            recording_id + "_" + topic_name_as_path + "_images_and_detections.bag");
    if (fs::exists(full_bag_path)) {
        ROS_ERROR_STREAM("File path already exists " + full_bag_path.string());
    }
    createDirectoryIfNotExists(full_bag_path);
    if (bag_.isOpen()) {
        bag_.close();
    }

    // Open the rosbag with the new name
    bag_.open(full_bag_path.string(), rosbag::bagmode::Write);
    return true;
}

void CameraDetectorNode::queryRecordingID(const ros::TimerEvent &event) {
    grand_tour_camera_detection_msgs::StartRecordingCalibrationDataService srv;

    // Wait for the service to be available with the specified timeout
    if (!start_recording_service_client_.waitForExistence(ros::Duration(2.0))) {
        if (bag_.isOpen()) {
            ROS_WARN_STREAM("Service: '" + recording_id_service_name_ + "' not available within the timeout period."
                                                                        " Closing bag with recording id: "
                            + recording_id_);
            bag_.close();
        }
        return;
    }

    // Call the service and check if it was successful
    if (start_recording_service_client_.call(srv)) {
        if (srv.response.recording_id.data != recording_id_) {
            recording_id_ = srv.response.recording_id.data;
            this->openNewBag(recording_id_);
            ROS_INFO_STREAM("Recording id set to: " + recording_id_);
        }
    } else {
        ROS_ERROR_STREAM("Failed to call recording id service " + recording_id_service_name_);
    }
}


// The image callback function that processes the incoming images
void CameraDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    images_received_++;

    cv::Mat image;
    try {
        // Convert the ROS image message to an OpenCV Mat using cv_bridge
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        image = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Process the image
    const auto validCorners = processImage(image);
    // If there were valid detections, increment the count
    if (!validCorners.empty()) {
        images_with_detections_++;
        if (bag_.isOpen()) {
            bag_.write(image_topic_, msg->header.stamp, msg);
        }
        // Publish detections as AprilGridDetections message
        this->publishDetections(validCorners, msg->header);
    }
}

std::map<int, Eigen::Vector2d> mapValidIDSToCorners(const Eigen::MatrixXd &corners, const std::vector<bool> &ids);

// This method processes an OpenCV image
std::map<int, Eigen::Vector2d> CameraDetectorNode::processImage(const cv::Mat &image) {
    Eigen::MatrixXd corners;
    std::vector<bool> ids;
    calibration_target_model_->computeObservation(image, corners, ids);
    const auto validCorners = mapValidIDSToCorners(corners, ids);
    return validCorners;
}

void CameraDetectorNode::publishDetections(const std::map<int, Eigen::Vector2d> &validCorners,
                                           const std_msgs::Header &header) {
    // Create the AprilGridDetections message
    grand_tour_camera_detection_msgs::CameraDetections detections_msg;

    // Fill in the header (assuming the frame ID and timestamp from the image)
    detections_msg.header = header;  // Adjust the frame ID as needed

    // Iterate through the valid corners and fill in the message
    for (const auto &[index, corner]: validCorners) {
        // 2D corners
        geometry_msgs::Point corner2d;
        corner2d.x = corner.x();
        corner2d.y = corner.y();
        corner2d.z = 0.0;  // Since it's a 2D corner, the z-component is 0
        detections_msg.corners2d.push_back(corner2d);

        // 3D model points
        geometry_msgs::Point model_point;
        Eigen::Vector3d model_pt = this->calibration_target_model_->point(index);  // Fetch the corresponding 3D point
        model_point.x = model_pt.x();
        model_point.y = model_pt.y();
        model_point.z = model_pt.z();
        detections_msg.modelpoint3d.push_back(model_point);

        // Corner IDs
        detections_msg.cornerids.push_back(index);
    }
    if (bag_.isOpen()) {
        bag_.write(detection_topic_, header.stamp, detections_msg);
    }
    // Publish the message
    detections_pub_.publish(detections_msg);
}

std::map<int, Eigen::Vector2d> mapValidIDSToCorners(const Eigen::MatrixXd &corners, const std::vector<bool> &ids) {
    std::map<int, Eigen::Vector2d> validCorners;

    // Loop through the ids vector to find valid corners
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i]) {
            Eigen::Vector2d corner(corners(i, 0), corners(i, 1));
            validCorners[i] = corner;
        }
    }

    return validCorners;
}

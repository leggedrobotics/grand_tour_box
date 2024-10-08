//
// Created by fu on 24/09/24.
//

#ifndef GRAND_TOUR_CAMERA_DETECTORS_DETECTOR_NODE_H
#define GRAND_TOUR_CAMERA_DETECTORS_DETECTOR_NODE_H

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include "calibrationtargetasl.h"
#include <Eigen/Dense>
#include <string>
#include <optional>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>

class CameraDetectorNode {
public:
    CameraDetectorNode(ros::NodeHandle &nh);

private:

    // Method to process an image
    std::map<int, Eigen::Vector2d> processImage(const cv::Mat &image);

    // Callback for the image topic
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    void publishDetections(const std::map<int, Eigen::Vector2d> &validCorners,
                           const std_msgs::Header &header);

    // Periodic log callback
    void logStatistics(const ros::TimerEvent &event);


    bool show_extraction_video_;
    int grid_size_x_;
    int grid_size_y_;
    double tag_size_;
    double tag_spacing_;

    ros::Subscriber image_sub_;  // Subscriber for the image topic
    ros::NodeHandle nh_;
    ros::Publisher detections_pub_;  // Publisher for the AprilGridDetections message
    std::unique_ptr<cameras::GridCalibrationTargetBase> calibration_target_model_;

    // Counters for received images and detected images
    int images_received_ = 0;
    int images_with_detections_ = 0;

    // Timer for logging statistics periodically
    ros::Timer log_timer_;
};


#endif //GRAND_TOUR_CAMERA_DETECTORS_DETECTOR_NODE_H

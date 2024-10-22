//
// Created by fu on 30/09/24.
//

#ifndef GRAND_TOUR_CERES_APPS_ROS_UTILS_H
#define GRAND_TOUR_CERES_APPS_ROS_UTILS_H


#include <string>
#include <chrono>
#include <utility>
#include <iostream>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <filesystem>

namespace fs = std::filesystem;

struct ScopedTimer {
    std::string name;  // Optional name for the timer
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;

    // Constructor: starts the timer and optionally accepts a name
    explicit ScopedTimer(std::string timer_name = "");

    std::chrono::duration<double> elapsed() const;
};

// Function to create the camera matrix and distortion coefficients matrices
void cameraParametersToCVMat(const CameraParameterPack &camera_parameters,
                             cv::Mat &cameraMatrix, cv::Mat &distCoeffs);

// Function to convert rvec, tvec (from OpenCV) to Eigen::Affine3d
bool rvecTvecToAffine3d(const cv::Mat &rvec, const cv::Mat &tvec, Eigen::Affine3d &output);


bool solvePnP(const CameraParameterPack &camera_parameters,
              const Eigen::Matrix2Xd &corners2d, const Eigen::Matrix3Xd &modelpoints3d,
              Eigen::Affine3d &output);

std::string getMatType(const cv::Mat &mat);

// Convert Eigen::Matrix3Xd to cv::Mat with CV_64FC3
cv::Mat eigen2cv_3d(const Eigen::Matrix3Xd &eigen_matrix);

// Convert Eigen::Matrix2Xd to cv::Mat with CV_64FC2
cv::Mat eigen2cv_2d(const Eigen::Matrix2Xd &eigen_matrix);

Observations2dModelPoints3dPointIDsPose3dSensorName
buildObservationFromRosMSG(const grand_tour_camera_detection_msgs::CameraDetections &camera_detections);

using stamp_t = unsigned long long;

std::set<stamp_t> findWithinTolerance(const std::set<stamp_t> &s, stamp_t element, stamp_t tolerance);

std::string getCurrentTimeFormatted();

#endif //GRAND_TOUR_CERES_APPS_ROS_UTILS_H

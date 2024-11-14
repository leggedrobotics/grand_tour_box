//
// Created by fu on 30/09/24.
//

#include <Eigen/Core>
#include "ros_utils.h" // Include the conversion header
#include <ros/ros.h>

std::chrono::duration<double> ScopedTimer::elapsed() const {
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    return elapsed;
}

ScopedTimer::ScopedTimer(std::string timer_name) : name(std::move(timer_name)),
                                                   start_time(std::chrono::high_resolution_clock::now()) {
}


Observations2dModelPoints3dPointIDsPose3dSensorName
buildObservationFromRosMSG(const grand_tour_camera_detection_msgs::CameraDetections &camera_detections) {
    int n_detections = camera_detections.corners2d.size();
    Observations2dModelPoints3dPointIDsPose3dSensorName observation;
    observation.observations2d = Eigen::Matrix2Xd(2, n_detections);
    observation.modelpoints3d = Eigen::Matrix3Xd(3, n_detections);
    observation.modelpointIDs = std::vector<unsigned int> (n_detections, 0);
    for (int i = 0; i < n_detections; i++) {
        observation.observations2d(0, i) = camera_detections.corners2d[i].x;
        observation.observations2d(1, i) = camera_detections.corners2d[i].y;

        observation.modelpoints3d(0, i) = camera_detections.modelpoint3d[i].x;
        observation.modelpoints3d(1, i) = camera_detections.modelpoint3d[i].y;
        observation.modelpoints3d(2, i) = camera_detections.modelpoint3d[i].z;

        observation.modelpointIDs[i] = camera_detections.cornerids[i];
    }
    observation.sensor_name = camera_detections.header.frame_id;
    return observation;
}


cv::Mat eigen2cv_2d(const Eigen::Matrix2Xd &eigen_matrix) {
    int cols = eigen_matrix.cols();

    // Create an OpenCV Mat with type CV_64FC2
    cv::Mat cv_mat(1, cols, CV_64FC2);

    // Iterate through the columns and assign the data to the OpenCV Mat
    for (int i = 0; i < cols; ++i) {
        cv_mat.at<cv::Vec2d>(0, i)[0] = eigen_matrix(0, i);  // First row of Eigen matrix -> first channel
        cv_mat.at<cv::Vec2d>(0, i)[1] = eigen_matrix(1, i);  // Second row -> second channel
    }

    return cv_mat;
}


cv::Mat eigen2cv_3d(const Eigen::Matrix3Xd &eigen_matrix) {
    int cols = eigen_matrix.cols();

    // Create an OpenCV Mat with type CV_64FC3
    cv::Mat cv_mat(1, cols, CV_64FC3);

    // Iterate through the columns and assign the data to the OpenCV Mat
    for (int i = 0; i < cols; ++i) {
        cv_mat.at<cv::Vec3d>(0, i)[0] = eigen_matrix(0, i);  // First row -> first channel
        cv_mat.at<cv::Vec3d>(0, i)[1] = eigen_matrix(1, i);  // Second row -> second channel
        cv_mat.at<cv::Vec3d>(0, i)[2] = eigen_matrix(2, i);  // Third row -> third channel
    }

    return cv_mat;
}


std::string getMatType(const cv::Mat &mat) {
    int type = mat.type();
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
        case CV_8U:
            r = "8U";
            break;
        case CV_8S:
            r = "8S";
            break;
        case CV_16U:
            r = "16U";
            break;
        case CV_16S:
            r = "16S";
            break;
        case CV_32S:
            r = "32S";
            break;
        case CV_32F:
            r = "32F";
            break;
        case CV_64F:
            r = "64F";
            break;
        default:
            r = "Unknown";
            break;
    }

    r += "C";
    r += std::to_string(chans);

    return r;
}


bool solvePnP(const CameraParameterPack &camera_parameters, const Eigen::Matrix2Xd &corners2d,
              const Eigen::Matrix3Xd &modelpoints3d, Eigen::Affine3d &output) {
    if (corners2d.cols() < 16) {
        return false;
    }
    // Convert Eigen matrices to cv::Mat using cv::eigen2cv
    cv::Mat K, D, rvec, tvec;
    cv::Mat cornersMat = eigen2cv_2d(corners2d);      // Eigen::Matrix2Xd to cv::Mat
    cv::Mat modelpointsMat = eigen2cv_3d(modelpoints3d);  // Eigen::Matrix3Xd to cv::Mat
    cameraParametersToCVMat(camera_parameters, K, D);
    try {
        if (camera_parameters.distortion_type == Distortion::Fisheye) {
            cv::Mat corners_undistorted;
            cv::fisheye::undistortPoints(cornersMat, corners_undistorted, K, D);
            cv::Mat eye = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat zero = cv::Mat::zeros(1, 4, CV_64F);
            cv::solvePnP(modelpointsMat.t(), corners_undistorted.t(), eye, zero, rvec, tvec);

        } else if (camera_parameters.distortion_type == Distortion::RadTan) {
            cv::solvePnP(modelpointsMat.t(), cornersMat.t(), K, D, rvec, tvec);
        } else {
            ROS_ERROR_STREAM("Received unexpected camera distortion model");
            return false;
        }
    } catch (const std::exception &e) {
        ROS_ERROR("Exception occurred while performing PnP: %s", e.what());
        ROS_ERROR_STREAM("Matrix type: " + getMatType(cornersMat));
    }
    rvecTvecToAffine3d(rvec, tvec, output);
    return true;
}

bool rvecTvecToAffine3d(const cv::Mat &rvec, const cv::Mat &tvec, Eigen::Affine3d &output) {
    // Convert rvec (rotation vector) to a rotation matrix using Rodrigues
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix);

    // Create an Eigen 3x3 matrix for rotation
    Eigen::Matrix3d rotationEigen;
    cv::cv2eigen(rotationMatrix, rotationEigen);  // Convert OpenCV rotation matrix to Eigen matrix

    // Create an Eigen 3x1 vector for translation
    Eigen::Vector3d translationEigen;
    translationEigen << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

    // Create an Eigen::Affine3d transformation matrix
    output = Eigen::Affine3d::Identity();
    output.linear() = rotationEigen;  // Set the rotation part
    output.translation() = translationEigen;  // Set the translation part
    return true;
}

void cameraParametersToCVMat(const CameraParameterPack &camera_parameters, cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
    // Create a 3x3 camera intrinsic matrix from the intrinsics array (fx, fy, cx, cy)
    cameraMatrix = (cv::Mat_<double>(3, 3) <<
                                           camera_parameters.fxfycxcy[0], 0, camera_parameters.fxfycxcy[2],  // fx, 0, cx
            0, camera_parameters.fxfycxcy[1], camera_parameters.fxfycxcy[3],  // 0, fy, cy
            0, 0, 1);                                                         // 0, 0, 1

    distCoeffs = cv::Mat(1, 4, CV_64F, const_cast<double *>(camera_parameters.dist_coeffs));
}

std::set<stamp_t> findWithinTolerance(const std::set<stamp_t> &s, stamp_t element, stamp_t tolerance) {
    std::set<stamp_t> result;

    // Get lower and upper bounds using the tolerance
    auto lower = s.lower_bound(element >= tolerance ? element - tolerance : 0); // Handle unsigned underflow
    auto upper = s.upper_bound(element + tolerance);

    // Collect all elements within [lower, upper) range
    for (auto it = lower; it != upper; ++it) {
        result.insert(*it);
    }

    return result;
}

std::string getCurrentTimeFormatted() {
    // Get the current time
    std::time_t now = std::time(nullptr);
    std::tm *localTime = std::localtime(&now);

    // Format the time as YYYY-MM-DD-HH-MM-SS
    std::ostringstream oss;
    oss << std::put_time(localTime, "%Y-%m-%d-%H-%M-%S");

    return oss.str();
}


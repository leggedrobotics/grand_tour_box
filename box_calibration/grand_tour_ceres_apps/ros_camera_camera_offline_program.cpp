//
// Created by fu on 27/09/24.
//


#include "ros_camera_camera_offline_program.h"
#include "ros_utils.h"
#include "ros_camera_camera_program.h"
#include <filesystem>

namespace fs = std::filesystem;

ROSCameraCameraOfflineProgram::ROSCameraCameraOfflineProgram(ROSCameraCameraParser parser) :
        ROSCameraCameraProgram(parser) {
    std::cout << parser.bag_paths.size() << std::endl;

    bool all_paths_valid = true;
    for (const auto &bag: parser.bag_paths) {
        if (!fs::exists(bag)) {
            std::cerr << "Error: File path does not exist: " << bag << std::endl;
            all_paths_valid = false;
        } else if (!fs::is_regular_file(bag)) {
            std::cerr << "Error: Path is not a regular file: " << bag << std::endl;
            all_paths_valid = false;
        }
        std::cerr << bag << std::endl;
    }

    is_valid = !parser.bag_paths.empty() && all_paths_valid;
    std::cout << is_valid << std::endl;
    bag_paths = parser.bag_paths;
    ready_for_extrinsics_ = true;
}

bool ROSCameraCameraOfflineProgram::publishDetectionsUsed(
        const grand_tour_camera_detection_msgs::CameraDetections &camera_detections) {
    return true;
}

void ROSCameraCameraOfflineProgram::run() {
    // load bags
    loadRosbagsIntoProgram();
    // set state from logged data

    std::map<std::string, int> total_edge_count = getTotalInAndOutExtrinsicEdges();
    for (const auto &[name, count]: total_edge_count) {
        std::cout << name << " has " << count << " in and out edges" << std::endl;
    }

    // solve
    this->setExtrinsicParametersVariableBeforeOpt();
    problem_->solver_options_.max_num_iterations = 200;
    ROS_DEBUG_STREAM("Solving...");
    {
        ScopedTimer timer;
        const bool success = this->Solve();
        this->rebuildProblemFromLoggedROSAlignmentData();
        this->Solve();
        std::cout << "Solve converged: " << std::boolalpha << success << std::endl;
        std::cout << "Computing covariances..." << std::endl;
        const auto covariances = this->computeCovariances();
        for (const auto &[name, covariance]: covariances) {
            std::cout << "Camera: " << name << std::endl;
            std::cout << covariance.rtvec_sigma.transpose() << std::endl;
            std::cout << covariance.fxfycxcy_sigma.transpose() << std::endl;
        }
        ROS_DEBUG("Covariance and ros publishing executed in: %f seconds", timer.elapsed().count());
    }
    // write results
    this->writeCalibrationOutput();
}

bool ROSCameraCameraOfflineProgram::loadRosbagsIntoProgram() {
    std::map<std::string, std::string> detectiontopic2imagetopic;
    for (const auto &[image_topic, _]: rostopic2frameid_) {
        const std::string detection_topic = image_topic + detection_suffix;
        detectiontopic2imagetopic[detection_topic] = image_topic;
    }
    for (const auto &bag_path: bag_paths) {
        try {
            rosbag::Bag bag;
            bag.open(bag_path, rosbag::bagmode::Read);
            std::cout << "Processing bag: " << bag_path << std::endl;

            // Set up a view for predefined topics only
            rosbag::View view(bag);
            for (const auto &connection_info: view.getConnections()) {
                const std::string &detection_topic = connection_info->topic;
                // Process only if the topic is in the predefined list
                if (detectiontopic2imagetopic.find(detection_topic) != rostopic2frameid_.end()) {
                    rosbag::View topic_view(bag, rosbag::TopicQuery(detection_topic));
                    for (const auto &m: topic_view) {
                        // Check if the message type is sensor_msgs/Image
                        grand_tour_camera_detection_msgs::CameraDetectionsConstPtr detection_msg =
                                m.instantiate<grand_tour_camera_detection_msgs::CameraDetections>();
                        if (detection_msg != nullptr) {
                            if (this->addAlignmentData(detection_msg->header.stamp,
                                                       *detection_msg,
                                                       true)) {
                                this->logged_ros_alignment_data_[detection_msg->header.frame_id][detection_msg->header.stamp.toNSec()] = *detection_msg;
                            }
                            calibration_time = detection_msg->header.stamp;
                        }
                    }
                }
            }

            bag.close();
        } catch (const rosbag::BagException &e) {
            std::cerr << "Error reading bag file " << bag_path << ": " << e.what() << std::endl;
        }
    }
    ROS_INFO_STREAM("Size: " + std::to_string(parsed_alignment_data.unique_timestamps.size()));
    return true;
}

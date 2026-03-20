//
// Created by fu on 27/09/24.
//

#include "ros_camera_camera_offline_program.h"

#include "ros_camera_camera_program.h"
#include "ros_utils.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

// --- Internal helpers (unnamed namespace) ------------------------------------
namespace {

bool validateBagPaths(const std::vector<std::string>& bag_paths) {
    if (bag_paths.empty()) {
        ROS_ERROR_STREAM("No bag paths provided.");
        return false;
    }

    bool all_ok = true;
    for (const auto& p : bag_paths) {
        const fs::path path{p};
        if (!fs::exists(path)) {
            ROS_ERROR_STREAM("File path does not exist: " << p);
            all_ok = false;
        } else if (!fs::is_regular_file(path)) {
            ROS_ERROR_STREAM("Path is not a regular file: " << p);
            all_ok = false;
        }
    }
    return all_ok;
}

// Build a map from detection topic to image topic we expect
    std::map<std::string, std::string>
    buildDetectionToImageMap(const std::map<std::string, std::string>& rostopic2frameid,
                             const std::string& detection_suffix) {
        std::map<std::string, std::string> detection_to_image;
        for (const auto& [image_topic, /*frame_id*/ _] : rostopic2frameid) {
            detection_to_image.emplace(image_topic + detection_suffix, image_topic);
        }
        return detection_to_image;
    }

} // namespace
// -----------------------------------------------------------------------------

ROSCameraCameraOfflineProgram::ROSCameraCameraOfflineProgram(ROSCameraCameraParser parser)
    : ROSCameraCameraProgram(parser) {

    // NB: base class likely copied parser fields we need (e.g., rostopic2frameid_).
    // If not, keep local copies here.
    bag_paths = parser.bag_paths;

    const bool paths_ok = validateBagPaths(bag_paths);
    is_valid = paths_ok;
    ready_for_extrinsics_ = true;

    ROS_INFO_STREAM("Offline program valid: " << std::boolalpha << is_valid);
}

bool ROSCameraCameraOfflineProgram::run() {
    if (!is_valid) {
        ROS_ERROR_STREAM("Program is not valid. Aborting run().");
        return false;
    }

    // 1) Load bags → fill logged data
    this->loadRosbagsIntoProgram();

    this->publishFrameTransforms();

    // 2) Introspect graph edges
    const auto total_edge_count = getTotalInAndOutExtrinsicEdges();
    for (const auto& [name, count] : total_edge_count) {
        ROS_INFO_STREAM(name << " has " << count << " in and out edges");
    }

    // 3) Solve (two passes, then covariance)
    setExtrinsicParametersVariableBeforeOpt();
    problem_->solver_options_.max_num_iterations = 500;

    ROS_INFO_STREAM("Solving...");
    {

        const bool success_first = Solve();
        // Rebuild from logged ROS alignment data and solve again
        this->rebuildProblemFromLoggedROSAlignmentData();
        const bool success_second = Solve();
        this->publishResiduals();
        this->publishFrameTransforms();

        ROS_INFO_STREAM("Solve converged (first pass): " << std::boolalpha << success_first);
        ROS_INFO_STREAM("Solve converged (second pass): " << std::boolalpha << success_second);

        ROS_INFO_STREAM("Computing covariances...");
        ScopedTimer timer;
        const auto covariances = computeCovariances();
        for (const auto& [name, covariance] : covariances) {
            ROS_INFO_STREAM("Camera: " << name);
            ROS_INFO_STREAM("  rtvec_sigma:      " << covariance.rtvec_sigma.transpose());
            ROS_INFO_STREAM("  fxfycxcy_sigma:   " << covariance.fxfycxcy_sigma.transpose());
        }

        ROS_INFO_STREAM("Covariance executed in " << timer.elapsed().count() << " seconds.");
    }

    // 4) Write results
    writeCalibrationOutput();

    return true;
}



void ROSCameraCameraOfflineProgram::loadRosbagsIntoProgram() {
    // Precompute the detection-topic → image-topic map
    const auto detection_to_image = buildDetectionToImageMap(rostopic2frameid_, detection_suffix);

    // Collect only the detection topics we care about (strings)
    std::vector<std::string> detection_topics;
    detection_topics.reserve(detection_to_image.size());
    for (const auto& [det_topic, _] : detection_to_image) {
        detection_topics.push_back(det_topic);
    }

    // Iterate bags
    size_t total_msgs = 0;
    for (const auto& bag_path : bag_paths) {
        try {
            ROS_INFO_STREAM("Processing bag: " << bag_path);
            rosbag::Bag bag;
            bag.open(bag_path, rosbag::bagmode::Read);

            // Query only on the detection topics of interest (saves time)
            rosbag::View view(bag, rosbag::TopicQuery(detection_topics));

            for (const auto& m : view) {
                const std::string& topic = m.getTopic();
                // Verify the topic is one we expect (NOTE: correct map checked here)
                if (detection_to_image.find(topic) == detection_to_image.end()) {
                    continue;
                }

                auto detection_msg =
                    m.instantiate<grand_tour_camera_detection_msgs::CameraDetections>();
                if (!detection_msg) {
                    continue;
                }

                const auto& stamp = detection_msg->header.stamp;
                const auto& frame_id = detection_msg->header.frame_id;

                // Add to alignment buffer + log if accepted
                if (addAlignmentData(stamp, *detection_msg, /*is_offline=*/true, false)) {
                    // NOTE: use nanoseconds as key; ensure map type is ordered or use unordered_map<uint64_t>
                    logged_ros_alignment_data_[frame_id][stamp.toNSec()] = *detection_msg;
                    calibration_time = stamp; // last stamp wins; OK for "latest"
                    ++total_msgs;
                }
            }

            bag.close();
        } catch (const rosbag::BagException& e) {
            ROS_ERROR_STREAM("Error reading bag file " << bag_path << ": " << e.what());
            // Continue with other bags instead of failing the whole run
        }
    }

    ROS_INFO_STREAM("Loaded " << total_msgs
                               << " detection messages across "
                               << bag_paths.size() << " bag(s).");
    ROS_INFO_STREAM("Unique timestamps buffered: "
                    << parsed_alignment_data.unique_timestamps.size());
}

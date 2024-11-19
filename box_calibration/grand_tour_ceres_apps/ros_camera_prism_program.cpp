#include "ros_camera_prism_program.h"
#include "ros_utils.h"
#include <gtboxcalibration/utils.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <grand_tour_camera_detection_msgs/CameraDetections.h>
#include <ap20_driver_ros/PositionDebug.h>

ROSCameraPrismProgram::ROSCameraPrismProgram(ROSCameraPrismParser parser) {
    const std::string detection_suffix = "_corner_detections";
    camera_packs = PopulateCameraParameterPacks(parser.cameras_calibration_path,
                                                parser.cameras_calibration_path);
    T_bundle_cam = FetchExtrinsicsFromYamlPath(parser.cameras_calibration_path);
    cam0_name = camera_packs.begin()->first;
    prism_board_in_total_station_params.t_offset[0] = 0;
    solve_time_offset = parser.solve_time_offset;
    output_yaml_path = parser.output_path;
    cameras_calibration_path = parser.cameras_calibration_path;
    std::map<std::string, std::string> detectiontopic2imagetopic;
    for (const auto &[image_topic, _]: camera_packs) {
        const std::string detection_topic = image_topic + detection_suffix;
        detectiontopic2imagetopic[detection_topic] = image_topic;
    }
    for (const auto &bag_path: parser.camera_bag_paths) {
        try {
            rosbag::Bag bag;
            bag.open(bag_path, rosbag::bagmode::Read);
            std::cout << "Processing bag: " << bag_path << std::endl;

            // Set up a view for predefined topics only
            rosbag::View view(bag);
            for (const auto &connection_info: view.getConnections()) {
                const std::string &detection_topic = connection_info->topic;
                // Process only if the topic is in the predefined list
                if (detectiontopic2imagetopic.find(detection_topic) != detectiontopic2imagetopic.end()) {
                    rosbag::View topic_view(bag, rosbag::TopicQuery(detection_topic));
                    for (const auto &m: topic_view) {
                        // Check if the message type is sensor_msgs/Image
                        grand_tour_camera_detection_msgs::CameraDetectionsConstPtr detection_msg =
                                m.instantiate<grand_tour_camera_detection_msgs::CameraDetections>();
                        if (detection_msg != nullptr) {
                            auto observation = buildObservationFromRosMSG(
                                    *detection_msg);
                            const auto camera_rostopic = detectiontopic2imagetopic.at(detection_topic);
                            Eigen::Affine3d T_camera_board;
                            solvePnP(camera_packs[camera_rostopic],
                                     observation.observations2d, observation.modelpoints3d,
                                     T_camera_board);
                            observation.T_sensor_model = T_camera_board;
                            unsigned long long stamp = detection_msg->header.stamp.toNSec();
                            camera_detections.unique_timestamps.insert(stamp);
                            camera_detections.observations[stamp][camera_rostopic] = observation;
                        }
                    }
                }
            }

            bag.close();
        } catch (const rosbag::BagException &e) {
            std::cerr << "Error reading bag file " << bag_path << ": " << e.what() << std::endl;
        }
    }

    SE3Transform se3Transform;
    se3Transform.assignToData(Eigen::Affine3d::Identity(),
                              prism_board_in_total_station_params.T_totalstation_board);
    try {
        rosbag::Bag bag;
        bag.open(parser.prism_bag_path, rosbag::bagmode::Read);
        std::cout << "Processing prism bag: " << parser.prism_bag_path << std::endl;

        // Set up a view for predefined topics only
        rosbag::View view(bag);
        for (const auto &connection_info: view.getConnections()) {
            const std::string &connection_topic = connection_info->topic;
            if (connection_topic != parser.prism_topic) continue;
            rosbag::View topic_view(bag, rosbag::TopicQuery(parser.prism_topic));
            for (const auto &m: topic_view) {
                // Check if the message type is sensor_msgs/Image
                ap20_driver_ros::PositionDebugConstPtr msg_ptr =
                        m.instantiate<ap20_driver_ros::PositionDebug>();
                if (msg_ptr != nullptr) {
                    const auto position_msg = msg_ptr->position;
                    Eigen::Vector3d position(position_msg.point.x, position_msg.point.y,
                                             position_msg.point.z);
                    prism_detections[msg_ptr->header.stamp.toNSec()] = position;
                }
            }
        }
        bag.close();
    } catch (const rosbag::BagException &e) {
        std::cerr << "Error reading bag file " << parser.prism_bag_path << ": " << e.what() << std::endl;
    }
    this->PopulateProblem();
}

bool ROSCameraPrismProgram::Solve() {
    bool success = CeresProgram::Solve();
    return success;
}

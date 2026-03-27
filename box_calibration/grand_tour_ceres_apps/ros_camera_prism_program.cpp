#include "ros_camera_prism_program.h"
#include "ros_utils.h"
#include <gtboxcalibration/utils.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <grand_tour_camera_detection_msgs/CameraDetections.h>
#include <ap20_driver_ros/PositionDebug.h>
#include <geometry_msgs/PointStamped.h>

void visualizePrismDetections(
        const std::unique_ptr<CameraPrismVizInterface> &viz, const PrismPositionDetectionData data) {
    std::vector<std::array<float, 3>> output;
    std::vector<double> timestamps;
    for (const auto &[time, position]: data) {
        timestamps.push_back(time / 1e9);
        auto position_f = position.cast<float>();
        output.push_back({position_f.x(), position_f.y(), position_f.z()});
    }
    viz->vizPrismObservedTrajectory(timestamps, output);
}

ROSCameraPrismProgram::ROSCameraPrismProgram(ROSCameraPrismParser parser,
                                             std::unique_ptr<CameraPrismVizInterface> viz) {
    viz_ = std::move(viz);
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
                            bool success = solvePnP(camera_packs[camera_rostopic],
                                                    observation.observations2d, observation.modelpoints3d,
                                                    T_camera_board);
                            if (!success) {
                                continue;
                            }
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
                geometry_msgs::PointStampedPtr msg_ptr =
                        m.instantiate<geometry_msgs::PointStamped>();
                if (msg_ptr != nullptr) {
                    const auto position_msg = msg_ptr->point;
                    Eigen::Vector3d position(position_msg.x, position_msg.y,
                                             position_msg.z);
                    prism_detections[msg_ptr->header.stamp.toNSec()] = position;
                }
            }
        }
        bag.close();
        if (viz_) {
            visualizePrismDetections(viz_, prism_detections);
        }
    } catch (const rosbag::BagException &e) {
        std::cerr << "Error reading bag file " << parser.prism_bag_path << ": " << e.what() << std::endl;
    }
    this->PopulateProblem();
}

void ROSCameraPrismProgram::visualizePerCameraPrismEstimates() {

    std::map<std::string, std::pair<std::vector<double>, std::vector<std::array<float, 3>>>> viz_data;

    const Eigen::Affine3d T_totalstation_board = SE3Transform::toEigenAffine(
            prism_board_in_total_station_params.T_totalstation_board);
    if (viz_) {
        viz_->vizStaticFrames(T_totalstation_board);
    }
    const Eigen::Vector3d t_cam0_prism = Eigen::Map<const Eigen::Vector3d>(
            prism_board_in_total_station_params.t_cam0_prism);
    const double t_offset = prism_board_in_total_station_params.t_offset[0];

    for (const auto &[camera_stamp, detections_at_stamp]: camera_detections.observations) {
        for (const auto &[camera_name, detection]: detections_at_stamp) {
            Eigen::Affine3d T_board_camera = detection.T_sensor_model.inverse();
            Eigen::Affine3d T_cam_cam0 = T_bundle_cam.at(camera_name).inverse() * T_bundle_cam.at(cam0_name);
            const Eigen::Vector3f cam_predicted_prism_in_total_station = (T_totalstation_board * T_board_camera
                    * T_cam_cam0 * t_cam0_prism).cast<float>();
            viz_data[camera_name].first.push_back(camera_stamp / 1e9);
            viz_data[camera_name].second.push_back({cam_predicted_prism_in_total_station.x(),
                                                    cam_predicted_prism_in_total_station.y(),
                                                    cam_predicted_prism_in_total_station.z()});;
        }
    }

    for (const auto &[camera_name, data]: viz_data) {
        viz_->vizPrismEstimatedByCameraDetections(camera_name, data.first, data.second);
    }

    const double time_offset = prism_board_in_total_station_params.t_offset[0];
    std::cout << "time offset: " << prism_board_in_total_station_params.t_offset[0] << std::endl;
    std::cout << "T_totalstation_board:\n" << T_totalstation_board.matrix() << std::endl;
    std::cout << "t_cam0_prism:\n" << t_cam0_prism.transpose() << std::endl;


    Eigen::Vector3d prism_position_sigma;
    Eigen::VectorXd board_pose_sigma;
    double time_offset_sigma;

    {
        std::vector<const double *> covariance_block = {prism_board_in_total_station_params.t_cam0_prism};
        Eigen::MatrixXd prism_position_covariance;
        problem_->ComputeAndFetchCovariance(
                covariance_block, prism_position_covariance);
        std::cout << "Prism position sigma:" << "\n" << prism_position_covariance.diagonal().array().sqrt().transpose()
                  << std::endl;
        prism_position_sigma = prism_position_covariance.diagonal().array().sqrt();
    }

    {
        std::vector<const double *> covariance_block = {prism_board_in_total_station_params.T_totalstation_board};
        Eigen::MatrixXd board_pose_covariance;
        problem_->ComputeAndFetchCovariance(
                covariance_block, board_pose_covariance);
        std::cout << "Board pose sigma:" << "\n" << board_pose_covariance.diagonal().array().sqrt().transpose()
                  << std::endl;
        board_pose_sigma = board_pose_covariance.diagonal().array().sqrt();
    }

    {
        std::vector<const double *> covariance_block = {prism_board_in_total_station_params.t_offset};
        Eigen::MatrixXd t_offset_covariance;
        problem_->ComputeAndFetchCovariance(
                covariance_block, t_offset_covariance);
        std::cout << "T-offset sigma:" << "\n" << t_offset_covariance.array().sqrt() << std::endl;
        time_offset_sigma = t_offset_covariance.array().sqrt()(0);
    }

    if (viz_){
        viz_->vizCalibrationResults(cam0_name, t_cam0_prism, time_offset, T_totalstation_board.matrix(),
                                    prism_position_sigma, board_pose_sigma, time_offset_sigma);
        const auto residuals_map3d = this->computeResidualMap3D();
        viz_->vizResiduals3D(residuals_map3d);
    }
}

bool ROSCameraPrismProgram::Solve() {
    bool success = CeresProgram::Solve();
    this->calculateResidualsAndFilterOutliers();
    this->PopulateProblem();
    this->visualizePerCameraPrismEstimates();
    success = CeresProgram::Solve();
    return success;
}

double median(std::vector<double> &vec) {
    size_t n = vec.size();
    if (n == 0) throw std::runtime_error("Empty vector");

    auto mid = vec.begin() + n / 2;
    std::nth_element(vec.begin(), mid, vec.end());

    if (n % 2 == 1) {
        return *mid;  // Odd case
    } else {
        int lower = *mid;
        std::nth_element(vec.begin(), mid - 1, vec.end());
        return (lower + *(mid - 1)) / 2.0;  // Even case
    }
}

void ROSCameraPrismProgram::calculateResidualsAndFilterOutliers() {
    std::vector<double> all_residuals;
    std::map<unsigned long long int, std::map<std::string, double>> residual_map = this->computeResidualNormMap(
            all_residuals);
    std::cout << "Median residual: " << median(all_residuals) << std::endl;

    const double median_residual = median(all_residuals);
    constexpr double outlier_median_threshold = 3;

    std::map<unsigned long long,
            std::map<std::string, Observations2dModelPoints3dPointIDsPose3dSensorName>> valid_observations;

    unsigned long n_filtered = 0;
    for (const auto &[camera_stamp, detections_at_stamp]: camera_detections.observations) {
        for (const auto &[camera_name, detection]: detections_at_stamp) {
            if (residual_map.contains(camera_stamp) && residual_map.at(camera_stamp).contains(camera_name)
                && residual_map.at(camera_stamp).at(camera_name) < outlier_median_threshold * median_residual) {
                valid_observations[camera_stamp][camera_name] = detection;
            } else {
                n_filtered++;
            }
        }
    }

    std::cout << "Filtered out " << n_filtered << " outliers greater than " << outlier_median_threshold <<
    " x Median."<< std::endl;
    camera_detections.observations = valid_observations;

}

std::map<unsigned long long int, std::map<std::string, double>>
ROSCameraPrismProgram::computeResidualNormMap(std::vector<double> &all_residuals) {
    std::map<unsigned long long, std::map<std::string, double>> residual_map;
    for (const auto &[camera_stamp, detections_at_stamp]: camera_detections.observations) {
        for (const auto &[camera_name, _]: detections_at_stamp) {
            std::vector<double> residuals(3, 0.0);  // Residuals of size 3

            bool success = problem_->getProblem().EvaluateResidualBlock(
                    residual_block_map[camera_name][camera_stamp], false, nullptr,
                    &residuals[0], nullptr);
            if (success && std::abs(residuals[0]) > 0) {
                double residual_norm = sqrt(residuals[0] * residuals[0] +
                                                 residuals[1] * residuals[1] +
                                                 residuals[2] * residuals[2]);
                residual_norm = residual_norm * prism_sigma_;
                all_residuals.push_back(residual_norm);
                residual_map[camera_stamp][camera_name] = residual_norm;
            } else {
            }
        }
    }
    return residual_map;
}

std::map<std::string, std::vector<Eigen::Vector3d>>
ROSCameraPrismProgram::computeResidualMap3D() {
    std::map<std::string, std::vector<Eigen::Vector3d>> residual_map;
    for (const auto &[camera_stamp, detections_at_stamp]: camera_detections.observations) {
        for (const auto &[camera_name, _]: detections_at_stamp) {
            std::vector<double> residuals(3, 0.0);  // Residuals of size 3

            bool success = problem_->getProblem().EvaluateResidualBlock(
                    residual_block_map[camera_name][camera_stamp], false, nullptr,
                    &residuals[0], nullptr);
            if (success && std::abs(residuals[0]) > 0) {
                Eigen::Vector3d residual_scaled = Eigen::Map<const Eigen::Vector3d>(residuals.data());
                residual_map[camera_name].push_back(residual_scaled * prism_sigma_);
            }
        }
    }
    return residual_map;
}


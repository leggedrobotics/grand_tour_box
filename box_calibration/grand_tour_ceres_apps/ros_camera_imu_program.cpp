#include "ros_camera_imu_program.h"
#include "ros_utils.h"
#include "imu_integrator.h"
#include <gtboxcalibration/utils.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <grand_tour_camera_detection_msgs/CameraDetections.h>

static constexpr char kDetectionSuffix[] = "_corner_detections";

ROSCameraIMUProgram::ROSCameraIMUProgram(ROSCameraIMUParser parser,
                                         std::unique_ptr<CameraImuVizInterface> viz) {
    viz_ = std::move(viz);

    camera_packs = PopulateCameraParameterPacks(parser.cameras_calibration_path,
                                                parser.cameras_calibration_path);
    T_bundle_cam = FetchExtrinsicsFromYamlPath(parser.cameras_calibration_path);
    cam0_name = camera_packs.begin()->first;
    solve_time_offset = parser.solve_time_offset;
    output_yaml_path = parser.output_path;
    cameras_calibration_path = parser.cameras_calibration_path;

    // Build the set of detection topics we expect (one per camera).
    std::vector<std::string> detection_topics;
    for (const auto &[camera_topic, _] : camera_packs)
        detection_topics.push_back(camera_topic + kDetectionSuffix);

    for (const auto &bag_path: parser.camera_bag_paths) {
        try {
            rosbag::Bag bag;
            bag.open(bag_path, rosbag::bagmode::Read);
            std::cout << "Processing bag: " << bag_path << std::endl;

            rosbag::View view(bag, rosbag::TopicQuery(detection_topics));
            for (const auto &m: view) {
                auto det_msg = m.instantiate<grand_tour_camera_detection_msgs::CameraDetections>();
                if (!det_msg || det_msg->cornerids.empty()) continue;

                // Strip suffix to recover the camera topic name.
                const std::string &det_topic = m.getTopic();
                const std::string camera_topic = det_topic.substr(
                        0, det_topic.size() - std::strlen(kDetectionSuffix));

                const double timestamp_s = det_msg->header.stamp.toSec();
                const size_t n = det_msg->cornerids.size();

                Eigen::Matrix2Xd detected_corners(2, n);
                Eigen::Matrix3Xd model_pts(3, n);
                for (size_t col = 0; col < n; ++col) {
                    detected_corners(0, col) = det_msg->corners2d[col].x;
                    detected_corners(1, col) = det_msg->corners2d[col].y;
                    model_pts(0, col) = det_msg->modelpoint3d[col].x;
                    model_pts(1, col) = det_msg->modelpoint3d[col].y;
                    model_pts(2, col) = det_msg->modelpoint3d[col].z;
                }

                Eigen::Affine3d T_camera_board;
                solvePnP(camera_packs.at(camera_topic), detected_corners, model_pts, T_camera_board);

                if (viz_) {
                    std::vector<std::array<float, 2>> corners_2d;
                    corners_2d.reserve(n);
                    for (size_t i = 0; i < n; ++i)
                        corners_2d.push_back({static_cast<float>(det_msg->corners2d[i].x),
                                              static_cast<float>(det_msg->corners2d[i].y)});
                    viz_->vizDetections(camera_topic, timestamp_s, corners_2d);
                    viz_->vizCameraPose(camera_topic, timestamp_s, T_camera_board);
                }
            }

            bag.close();
        } catch (const rosbag::BagException &e) {
            std::cerr << "Error reading bag file " << bag_path << ": " << e.what() << std::endl;
        }
    }

//    SE3Transform se3Transform;
//    se3Transform.assignToData(Eigen::Affine3d::Identity(),
//                              prism_board_in_total_station_params.T_totalstation_board);
    try {
        rosbag::Bag bag;
        bag.open(parser.imu_bag_path, rosbag::bagmode::Read);
        std::cout << "Processing imu bag: " << parser.imu_bag_path << std::endl;

        // Set up a view for predefined topics only
        rosbag::View view(bag);
        for (const auto &connection_info: view.getConnections()) {
            const std::string &connection_topic = connection_info->topic;
            if (connection_topic != parser.imu_topic) continue;
            rosbag::View topic_view(bag, rosbag::TopicQuery(parser.imu_topic));

            ImuIntegrator integrator({
                .position    = Eigen::Vector3d::Zero(),
                .velocity    = Eigen::Vector3d::Zero(),
                .orientation = Eigen::Quaterniond::Identity(),
                .acc_bias    = Eigen::Vector3d::Zero(),
                .gyro_bias   = Eigen::Vector3d::Zero(),
                .timestamp_s = -1.0,
            });

            for (const auto &m: topic_view) {
                sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                if (imu_msg == nullptr) continue;

                const double timestamp_s = imu_msg->header.stamp.toSec();
                const Eigen::Vector3d gyro{imu_msg->angular_velocity.x,
                                           imu_msg->angular_velocity.y,
                                           imu_msg->angular_velocity.z};
                const Eigen::Vector3d accel{imu_msg->linear_acceleration.x,
                                            imu_msg->linear_acceleration.y,
                                            imu_msg->linear_acceleration.z};

                // Seed timestamp on first message, then integrate.
                if (integrator.state().timestamp_s < 0.0) {
                    ImuIntegrator::State seed = integrator.state();
                    seed.timestamp_s = timestamp_s;
                    integrator = ImuIntegrator(seed);
                    continue;
                }

                integrator.integrate(timestamp_s, gyro, accel);

                if (viz_) {
                    const auto& s = integrator.state();
                    viz_->vizImuState(timestamp_s, s.position, s.velocity,
                                      s.orientation, s.acc_bias, s.gyro_bias);
                }
            }
        }
        bag.close();
    } catch (const rosbag::BagException &e) {
        std::cerr << "Error reading bag file " << parser.imu_bag_path << ": " << e.what() << std::endl;
    }
}


bool ROSCameraIMUProgram::Solve() {
    bool success = CeresProgram::Solve();
    this->calculateResidualsAndFilterOutliers();
    this->PopulateProblem();
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

void ROSCameraIMUProgram::calculateResidualsAndFilterOutliers() {
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
              " x Median." << std::endl;
    camera_detections.observations = valid_observations;

}

std::map<unsigned long long int, std::map<std::string, double>>
ROSCameraIMUProgram::computeResidualNormMap(std::vector<double> &all_residuals) {
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
ROSCameraIMUProgram::computeResidualMap3D() {
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


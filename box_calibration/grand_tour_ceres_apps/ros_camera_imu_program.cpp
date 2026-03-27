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
    imu_topic = parser.imu_topic;

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

                auto observation = buildObservationFromRosMSG(
                        *det_msg);

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
                bool success = solvePnP(
                        camera_packs.at(camera_topic), detected_corners, model_pts, T_camera_board);

                if (!success) {
                    continue;
                }
                observation.T_sensor_model = T_camera_board;
                unsigned long long stamp = det_msg->header.stamp.toNSec();
                camera_detections.unique_timestamps.insert(stamp);
                camera_detections.observations[stamp][camera_topic] = observation;

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

                imu_observations.push_back({gyro, accel, timestamp_s});
            }
        }
        bag.close();
    } catch (const rosbag::BagException &e) {
        std::cerr << "Error reading bag file " << parser.imu_bag_path << ": " << e.what() << std::endl;
    }

    // --- Initialize parameters ---

    // T_camera_bundle_imu starts as identity.
    SE3Transform::assignToData(Eigen::Affine3d::Identity(), T_camera_bundle_imu);

    std::cout << "Loaded " << camera_detections.unique_timestamps.size()
              << " keyframes, " << imu_observations.size() << " IMU observations." << std::endl;

    this->PopulateProblem();
}


bool ROSCameraIMUProgram::Solve() {
    PreSolveExtrinsic();
    PreSolveBoard();
    bool success = CeresProgram::Solve();
    WriteOutputParameters();

    if (viz_) {
        // Static sensor frames in the bundle.
        std::map<std::string, Eigen::Affine3d> T_bundle_cameras;
        for (const auto& [cam_name, cam_pack] : camera_packs)
            T_bundle_cameras[cam_name] = SE3Transform::toEigenAffine(cam_pack.T_bundle_sensor);
        for (const auto stamp : camera_detections.unique_timestamps) {
            for (const auto& [cam_name, det] : camera_detections.observations.at(stamp)) {
                // Board frame is the world frame (identity), so T_world_camera = T_board_camera.
                const Eigen::Affine3d T_board_camera = det.T_sensor_model.inverse();
                const Eigen::Affine3d T_world_camera = T_board_camera;
                viz_->vizCameraPose3D(cam_name, static_cast<double>(stamp) * 1e-9, T_world_camera);
                const Eigen::Affine3d T_world_bundle = T_world_camera * SE3Transform::toEigenAffine(
                        camera_packs.at(cam0_name).T_bundle_sensor).inverse();
                viz_->vizDetectionPoints3D(cam_name, static_cast<double>(stamp) * 1e-9,
                                           T_world_camera, det.T_sensor_model * det.modelpoints3d);
            }
        }
        const Eigen::Affine3d T_bundle_imu = SE3Transform::toEigenAffine(T_camera_bundle_imu);

        // Optimized keyframe IMU trajectory.
        for (const auto& [stamp, params] : keyframe_params) {
            const Eigen::Affine3d T_board_imu = SE3Transform::toEigenAffine(params.T_board_imu);
            const double t = static_cast<double>(stamp) * 1e-9;
            viz_->vizRigPose3D(t, T_board_imu);
            viz_->vizImuPoseTrajectory("keyframe", t, T_board_imu);
        }

        // Camera-chain IMU trajectory: T_board_imu^{cam} = inv(T_bundle_board_k) * T_bundle_imu.
        for (const auto& [stamp, detections] : camera_detections.observations) {
            for (const auto& [cam_name, det] : detections) {
                const Eigen::Affine3d T_bundle_sensor = SE3Transform::toEigenAffine(
                        camera_packs.at(cam_name).T_bundle_sensor);
                const Eigen::Affine3d T_bundle_board = T_bundle_sensor * det.T_sensor_model;
                const Eigen::Affine3d T_board_imu_cam = T_bundle_board.inverse() * T_bundle_imu;
                viz_->vizImuPoseTrajectory("camera_chain", static_cast<double>(stamp) * 1e-9, T_board_imu_cam);
                break;  // one camera per stamp is sufficient
            }
        }
        viz_->vizExtrinsics(T_bundle_cameras, SE3Transform::toEigenAffine(T_camera_bundle_imu));
        viz_->vizBoardPose3D(Eigen::Affine3d::Identity());

        // Relative board-point errors (metres RMS per point) per keyframe interval.
        for (const auto& [stamp_k, block_id] : relative_residual_block_map) {
            const int n = problem_->getProblem().GetCostFunctionForResidualBlock(block_id)->num_residuals();
            const int n_points = (n - 9) / 3;  // residual layout: 3*n_pts + 9 (SE3 + vel continuity)
            std::vector<double> residuals(n);
            problem_->getProblem().EvaluateResidualBlock(block_id, false, nullptr, residuals.data(), nullptr);
            double sum_sq = 0;
            for (int i = 0; i < n_points; ++i)
                sum_sq += residuals[i*3]*residuals[i*3]
                        + residuals[i*3+1]*residuals[i*3+1]
                        + residuals[i*3+2]*residuals[i*3+2];
            viz_->vizWorldFramePointError("relative",
                                          static_cast<double>(stamp_k) * 1e-9,
                                          std::sqrt(sum_sq / n_points));
        }
    }

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

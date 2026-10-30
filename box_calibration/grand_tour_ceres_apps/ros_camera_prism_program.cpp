#include "ros_camera_prism_program.h"
#include "ros_utils.h"
#include <gtboxcalibration/prismpositionresiduals.h>
#include <ap20_driver_ros/PositionDebug.h>
#include <geometry_msgs/PointStamped.h>
#include <grand_tour_camera_detection_msgs/CameraDetections.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <limits>

ROSCameraPrismProgram::ROSCameraPrismProgram(const ROSCameraPrismParser& parser,
                                             std::unique_ptr<CameraPrismVizInterface> viz)
    : viz_(std::move(viz)) {
    camera_packs = PopulateCameraParameterPacks(parser.cameras_calibration_path, parser.cameras_calibration_path);
    T_bundle_cam = FetchExtrinsicsFromYamlPath(parser.cameras_calibration_path);
    if (camera_packs.empty() || T_bundle_cam.empty()) throw std::invalid_argument("No camera calibration");
    cam0_name = camera_packs.begin()->first;
    solve_time_offset = parser.solve_time_offset;
    output_yaml_path = parser.output_path;
    cameras_calibration_path = parser.cameras_calibration_path;
    std::vector<std::string> topics;
    std::map<std::string, std::string> topic_to_camera, frames;
    for (const auto& [camera, pack] : camera_packs) {
        topics.push_back(camera + "_corner_detections");
        topic_to_camera[topics.back()] = camera;
    }
    const auto config = YAML::LoadFile(cameras_calibration_path);
    for (const auto& entry : config)
        if (entry.second["frame_id"])
            frames[entry.second["rostopic"].as<std::string>()] = entry.second["frame_id"].as<std::string>();
    for (const auto& path : parser.camera_bag_paths) {
        rosbag::Bag bag(path, rosbag::bagmode::Read);
        for (const auto& message : rosbag::View(bag, rosbag::TopicQuery(topics))) {
            const auto msg = message.instantiate<grand_tour_camera_detection_msgs::CameraDetections>();
            if (!msg) throw std::invalid_argument("Unexpected camera detection message type");
            const auto& camera = topic_to_camera.at(message.getTopic());
            if (frames.count(camera) && frames.at(camera) != msg->header.frame_id)
                throw std::invalid_argument("Inconsistent camera frame: " + camera);
            frames[camera] = msg->header.frame_id;
            if (msg->cornerids.empty()) continue;
            auto observation = buildObservationFromRosMSG(*msg);
            Eigen::Affine3d pose;
            if (!solvePnP(camera_packs.at(camera), observation.observations2d, observation.modelpoints3d, pose)) continue;
            if (!pose.matrix().allFinite()) throw std::invalid_argument("Nonfinite camera pose");
            observation.T_sensor_model = pose;
            const auto stamp = msg->header.stamp.toNSec();
            if (camera_detections.observations[stamp].count(camera))
                throw std::invalid_argument("Duplicate camera detection timestamp");
            camera_detections.observations[stamp][camera] = observation;
            camera_detections.unique_timestamps.insert(stamp);
        }
    }
    rosbag::Bag bag(parser.prism_bag_path, rosbag::bagmode::Read);
    std::string prism_frame;
    bool have_frame = false;
    for (const auto& message : rosbag::View(bag, rosbag::TopicQuery(parser.prism_topic))) {
        geometry_msgs::Point point;
        std_msgs::Header header;
        if (const auto msg = message.instantiate<ap20_driver_ros::PositionDebug>()) {
            point = msg->position.point;
            header = msg->header;  // Preserve the timestamp convention used by the existing AP20 path.
        } else if (const auto msg = message.instantiate<geometry_msgs::PointStamped>()) {
            point = msg->point;
            header = msg->header;
        } else {
            throw std::invalid_argument("Prism topic must contain PositionDebug or PointStamped");
        }
        if (have_frame && prism_frame != header.frame_id) throw std::invalid_argument("Changing prism frame");
        prism_frame = header.frame_id;
        have_frame = true;
        const Eigen::Vector3d position(point.x, point.y, point.z);
        if (!position.allFinite()) throw std::invalid_argument("Nonfinite prism position");
        if (!prism_detections.emplace(header.stamp.toNSec(), position).second)
            throw std::invalid_argument("Duplicate prism timestamp");
    }
    SE3Transform::assignToData(Eigen::Affine3d::Identity(), prism_board_in_total_station_params.T_totalstation_board);
    if (!PopulateProblem()) throw std::invalid_argument("Insufficient overlapping camera/prism observations");
}

bool ROSCameraPrismProgram::Solve() {
    if (!CameraPrismProgram::Solve()) return false;
    if (viz_) visualize();
    return true;
}

void ROSCameraPrismProgram::visualize() {
    if (!viz_) return;
    const auto transform = SE3Transform::toEigenAffine(prism_board_in_total_station_params.T_totalstation_board);
    const Eigen::Map<const Eigen::Vector3d> lever(prism_board_in_total_station_params.t_cam0_prism);
    const double offset = prism_board_in_total_station_params.t_offset[0];
    viz_->vizStaticFrames(transform);
    std::vector<double> stamps;
    std::vector<std::array<float, 3>> positions;
    for (const auto& [stamp, point] : prism_detections) {
        stamps.push_back(stamp * 1e-9);
        positions.push_back({float(point.x()), float(point.y()), float(point.z())});
    }
    viz_->vizPrismObservedTrajectory(stamps, positions);
    std::map<std::string, std::vector<Eigen::Vector3d>> residuals;
    std::map<unsigned long long, Eigen::Vector3d> sampled_prism;
    for (const auto& [camera, blocks] : residual_block_map) {
        stamps.clear();
        positions.clear();
        std::vector<Eigen::Affine3d> camera_poses;
        const auto camera_to_origin = T_bundle_cam.at(camera).inverse() * T_bundle_cam.at(cam0_name);
        for (const auto& [stamp, block] : blocks) {
            Eigen::Vector3d residual;
            if (!problem_->getProblem().EvaluateResidualBlock(block, false, nullptr, residual.data(), nullptr))
                throw std::runtime_error("Cannot evaluate prism residual for visualization");
            residual *= prism_sigma;
            residuals[camera].push_back(residual);
            const Eigen::Affine3d camera_pose = transform * camera_detections.observations.at(stamp).at(camera).T_sensor_model.inverse();
            const Eigen::Vector3d predicted = camera_pose * camera_to_origin * lever;
            const Eigen::Vector3d observed = predicted + residual;
            // Estimated and observed positions share the corrected prism timeline.
            stamps.push_back(stamp * 1e-9 + offset);
            positions.push_back({float(predicted.x()), float(predicted.y()), float(predicted.z())});
            sampled_prism[stamp] = observed;
            camera_poses.push_back(camera_pose);
        }
        viz_->vizPrismEstimatedByCameraDetections(camera, stamps, positions);
        viz_->vizCameraTrajectory(camera, stamps, camera_poses);
    }
    stamps.clear();
    positions.clear();
    for (const auto& [stamp, observed] : sampled_prism) {
        stamps.push_back(stamp * 1e-9 + offset);
        positions.push_back({float(observed.x()), float(observed.y()), float(observed.z())});
    }
    viz_->vizPrismInterpolatedAtCameraTime(stamps, positions);
    const Eigen::VectorXd sigmas = calibration_covariance.diagonal().array().sqrt();
    viz_->vizCalibrationResults(cam0_name, lever, offset, transform.matrix(), sigmas.segment<3>(6),
                                sigmas.head(6), solve_time_offset ? sigmas(9) : std::numeric_limits<double>::quiet_NaN());
    viz_->vizResiduals3D(residuals);
}

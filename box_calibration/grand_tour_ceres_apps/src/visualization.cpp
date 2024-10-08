//
// Created by fu on 09/09/2024.
//

#include <gtboxcalibration/visualization.h>

void CameraPrismVisualization::PlotState(const std::string suffix) {
    Eigen::Map<const Eigen::Vector3d> t_cam0_prism(program_.prism_board_in_total_station_params.t_cam0_prism);
    Eigen::Affine3d T_leica_board = Eigen::Affine3d(
            SE3Transform::toEigen(program_.prism_board_in_total_station_params.T_totalstation_board));

    std::vector<Eigen::Vector3f> prism_positions_measured;
    for (const auto&[stamp, prism_position]: program_.prism_detections) {
        rec.set_time_nanos("ros_time", stamp);
        prism_positions_measured.emplace_back(prism_position.cast<float>());
        rec.log(root_name_ + "/x/leica_measured", rerun::Scalar(prism_position.x()));
        rec.log(root_name_ + "/y/leica_measured", rerun::Scalar(prism_position.y()));
        rec.log(root_name_ + "/z/leica_measured", rerun::Scalar(prism_position.z()));
    }
    rec.log(root_name_ + "/leica_measured_positions", rerun::Points3D(prism_positions_measured));

    std::vector<Eigen::Vector3f> prism_positions_predicted;
    for (const auto &camera_stamp: program_.camera_detections.unique_timestamps) {
        for (const auto&[cam_name, cam_detection]: program_.camera_detections.observations.at(camera_stamp)) {
            Eigen::Affine3d T_board_cam = cam_detection.T_sensor_model.inverse();
            Eigen::Affine3d T_cam_cam0 =
                    program_.T_bundle_cam.at(cam_name).inverse() * program_.T_bundle_cam.at(program_.cam0_name);
            Eigen::Vector3d predicted_prism_in_leica = T_leica_board * T_board_cam * T_cam_cam0 * t_cam0_prism;
            prism_positions_predicted.emplace_back(predicted_prism_in_leica.cast<float>());
            rec.set_time_nanos("ros_time", camera_stamp + static_cast<long long>(
                    program_.prism_board_in_total_station_params.t_offset[0] * 1e9));
            rec.log(root_name_ + "/x/" + suffix + cam_name, rerun::Scalar(predicted_prism_in_leica.x()));
            rec.log(root_name_ + "/y/" + suffix + cam_name, rerun::Scalar(predicted_prism_in_leica.y()));
            rec.log(root_name_ + "/z/" + suffix + cam_name, rerun::Scalar(predicted_prism_in_leica.z()));
        }
    }
    rec.log(root_name_ + "/predicted_positions/" + suffix, rerun::Points3D(prism_positions_predicted));
}

void CameraPrismVisualization::PlotResiduals(
        const std::map<std::string, std::map<unsigned long long, std::pair<Eigen::Vector3d, double>>> &residuals_and_losses) {
    std::map<std::string, std::vector<Eigen::Vector3f>> residuals;
    for (const auto &[cam_name, residual_loss_at_stamp]: residuals_and_losses) {
        for (const auto&[stamp, residual_loss]: residual_loss_at_stamp) {
            rec.set_time_nanos("ros_time", stamp);
            rec.log("loss/" + cam_name, rerun::Scalar(residual_loss.second));
            residuals[cam_name].emplace_back(residual_loss.first.cast<float>());
        }
    }
    for (const auto &[cam_name, residuals_for_camera]: residuals) {
        rec.log(cam_name + "/residuals", rerun::Points3D(residuals_for_camera));
    }
}

CameraPrismVisualization::CameraPrismVisualization(const std::string &rootName, const std::string &windowName,
                                                   const CameraPrismProgram &program)
        : Visualization(rootName, windowName), program_(program) {}

void CameraPrismVisualization::PlotCovariance() {
    this->program_.problem_->PlotCovariance();
}


Visualization::Visualization(const std::string &rootName, const std::string &windowName) : root_name_(rootName),
                                                                                           window_name_(windowName),
                                                                                           rec(rerun::RecordingStream(
                                                                                                   windowName)) {
    rec.spawn().exit_on_failure();
}

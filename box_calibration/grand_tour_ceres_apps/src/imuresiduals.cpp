#include <gtboxcalibration/imuresiduals.h>

RelativePoseExtrinsicError::RelativePoseExtrinsicError(Eigen::Matrix3d R_bundle_sensor_inv,
                                                       Eigen::Matrix3d R_imu_rel,
                                                       Eigen::Matrix3d R_cam_rel_obs)
        : R_bundle_sensor_inv_(std::move(R_bundle_sensor_inv)),
          R_imu_rel_(std::move(R_imu_rel)),
          R_cam_rel_obs_(std::move(R_cam_rel_obs)) {}

ceres::CostFunction *RelativePoseExtrinsicError::Create(Eigen::Matrix3d R_bundle_sensor_inv,
                                                        Eigen::Matrix3d R_imu_rel,
                                                        Eigen::Matrix3d R_cam_rel_obs) {
    return new ceres::AutoDiffCostFunction<RelativePoseExtrinsicError, 3,
            SE3Transform::NUM_PARAMETERS>(
            new RelativePoseExtrinsicError(std::move(R_bundle_sensor_inv),
                                           std::move(R_imu_rel),
                                           std::move(R_cam_rel_obs)));
}

VisualReprojectionError::VisualReprojectionError(Eigen::Matrix3Xd model_points,
                                                 Eigen::Matrix2Xd observed_points,
                                                 CameraParameterPack camera_parameters,
                                                 PinholeCamera camera)
        : model_points_at_k_(std::move(model_points)),
          observed_points_at_k_(std::move(observed_points)),
          camera_parameters_(std::move(camera_parameters)),
          camera_(std::move(camera)) {}

ceres::CostFunction *VisualReprojectionError::Create(Eigen::Matrix3Xd model_points,
                                                     Eigen::Matrix2Xd observed_points,
                                                     CameraParameterPack camera_parameters,
                                                     PinholeCamera camera) {
    const int n_residuals = static_cast<int>(observed_points.cols()) * 2;
    return new ceres::AutoDiffCostFunction<VisualReprojectionError, ceres::DYNAMIC,
            SE3Transform::NUM_PARAMETERS,   // T_camera_bundle_imu
            SE3Transform::NUM_PARAMETERS,   // T_world_board
            SE3Transform::NUM_PARAMETERS>(  // T_world_imu_k
            new VisualReprojectionError(std::move(model_points), std::move(observed_points),
                                        std::move(camera_parameters), std::move(camera)),
            n_residuals);
}

IMUConsistencyError::IMUConsistencyError(std::vector<IMUObservation> imu_observations,
                                         double time_k_secs,
                                         double time_kp1_secs)
        : imu_observations_k_to_kp1_(std::move(imu_observations)),
          time_k_secs_(time_k_secs),
          time_kp1_secs_(time_kp1_secs) {}

ceres::CostFunction *IMUConsistencyError::Create(std::vector<IMUObservation> imu_observations,
                                                 double time_k_secs,
                                                 double time_kp1_secs) {
    return new ceres::AutoDiffCostFunction<IMUConsistencyError, 9,
            SE3Transform::NUM_PARAMETERS,   // T_world_imu_k
            3,                              // v_world_imu_k
            3,                              // bias_gyro_k
            3,                              // bias_accel_k
            SE3Transform::NUM_PARAMETERS,   // T_world_imu_kp1
            3,                              // v_world_imu_kp1
            3>(                             // gravity_world
            new IMUConsistencyError(std::move(imu_observations), time_k_secs, time_kp1_secs));
}
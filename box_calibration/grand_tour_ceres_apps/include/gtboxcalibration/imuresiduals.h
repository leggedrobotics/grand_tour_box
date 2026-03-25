//
// Created by fu on 25/03/26.
//

#ifndef GRAND_TOUR_CERES_APPS_IMURESIDUALS_H
#define GRAND_TOUR_CERES_APPS_IMURESIDUALS_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <gtboxcalibration/datatypes.h>
#include <gtboxcalibration/camerageometry.h>
#include <gtboxcalibration/parameterhelpers.h>
#include <sophus/so3.hpp>


// ---------------------------------------------------------------------------
// Hand-eye pre-solve residual: rotation-only relative pose constraint.
// Compares the visual relative rotation between consecutive keyframes k→k+1
// to the IMU-integrated relative rotation over the same interval.
// Only T_camera_bundle_imu is a parameter; everything else is baked in.
//
// Parameters:
//   params_T_camera_bundle_imu  [7]  — extrinsic: IMU → camera bundle
// Residual size: 3  (skew-symmetric part of R_err)
// ---------------------------------------------------------------------------
class RelativePoseExtrinsicError {
public:
    RelativePoseExtrinsicError(Eigen::Matrix3d R_bundle_sensor_inv,
                               Eigen::Matrix3d R_imu_rel,
                               Eigen::Matrix3d R_cam_rel_obs);

    static ceres::CostFunction *Create(Eigen::Matrix3d R_bundle_sensor_inv,
                                       Eigen::Matrix3d R_imu_rel,
                                       Eigen::Matrix3d R_cam_rel_obs);

    template<typename T>
    bool operator()(const T *const params_T_camera_bundle_imu, T *residual) const {
        // Extract rotation from SE3 quaternion block.
        Eigen::Matrix<T, 3, 3> R_camera_bundle_imu = Eigen::Matrix<T, 3, 3>::Identity();
        ceres::QuaternionToRotation(params_T_camera_bundle_imu,
                                    ceres::ColumnMajorAdapter3x3(R_camera_bundle_imu.data()));

        // R_cam_imu = R_bundle_sensor^{-1} * R_camera_bundle_imu
        // maps from IMU orientation to camera sensor orientation.
        Eigen::Matrix<T, 3, 3> R_cam_imu =
                R_bundle_sensor_inv_.cast<T>() * R_camera_bundle_imu;

        // Predicted relative camera rotation from IMU integration.
        Eigen::Matrix<T, 3, 3> R_pred =
                R_cam_imu * R_imu_rel_.cast<T>() * R_cam_imu.transpose();

        // Residual: skew-symmetric part of (R_pred^T * R_obs - I).
        Eigen::Matrix<T, 3, 3> R_err = R_pred.transpose() * R_cam_rel_obs_.cast<T>();
        residual[0] = R_err(2, 1) - R_err(1, 2);
        residual[1] = R_err(0, 2) - R_err(2, 0);
        residual[2] = R_err(1, 0) - R_err(0, 1);
        return true;
    }

    Eigen::Matrix3d R_bundle_sensor_inv_;
    Eigen::Matrix3d R_imu_rel_;
    Eigen::Matrix3d R_cam_rel_obs_;
};

// ---------------------------------------------------------------------------
// Visual reprojection error at a single keyframe k.
// Parameters:
//   params_T_camera_bundle_imu  [6]  — extrinsic: IMU → camera bundle
//   params_T_world_board        [6]  — board pose in world
//   params_T_world_imu_k        [6]  — IMU pose at keyframe k
// Residual size: 2 * n_observed_points
// ---------------------------------------------------------------------------
class VisualReprojectionError {
public:
    VisualReprojectionError(Eigen::Matrix3Xd model_points,
                            Eigen::Matrix2Xd observed_points,
                            CameraParameterPack camera_parameters,
                            PinholeCamera camera);

    static ceres::CostFunction *Create(Eigen::Matrix3Xd model_points,
                                       Eigen::Matrix2Xd observed_points,
                                       CameraParameterPack camera_parameters,
                                       PinholeCamera camera);

    template<typename T>
    bool operator()(
            const T *const params_T_camera_bundle_imu,
            const T *const params_T_world_board,
            const T *const params_T_world_imu_k,
            T *residual) const {

        EigenJetAffine<T> T_camera_bundle_imu = SE3Transform::toEigenAffineJetSafe(params_T_camera_bundle_imu);
        EigenJetAffine<T> T_world_board        = SE3Transform::toEigenAffineJetSafe(params_T_world_board);
        EigenJetAffine<T> T_world_imu_k        = SE3Transform::toEigenAffineJetSafe(params_T_world_imu_k);
        Eigen::Affine3d T_camerabundle_cam   = SE3Transform::toEigenAffine(
                camera_parameters_.T_bundle_sensor);

        Eigen::Matrix<T, 3, Eigen::Dynamic> model_points = model_points_at_k_.cast<T>();

        Eigen::Matrix<T, 3, Eigen::Dynamic> points_in_camera =
                T_camerabundle_cam.cast<T>().inverse() *
                T_camera_bundle_imu *
                T_world_imu_k.inverse() *
                T_world_board * model_points;

        T fxfycxcy[4]    = {T(camera_parameters_.fxfycxcy[0]),    T(camera_parameters_.fxfycxcy[1]),
                            T(camera_parameters_.fxfycxcy[2]),    T(camera_parameters_.fxfycxcy[3])};
        T dist_coeffs[4] = {T(camera_parameters_.dist_coeffs[0]), T(camera_parameters_.dist_coeffs[1]),
                            T(camera_parameters_.dist_coeffs[2]), T(camera_parameters_.dist_coeffs[3])};

        Eigen::Matrix<T, 2, Eigen::Dynamic> projected = camera_.project(
                fxfycxcy, dist_coeffs, points_in_camera);

        Eigen::Matrix<T, 2, Eigen::Dynamic> errors = observed_points_at_k_.cast<T>() - projected;
        for (int i = 0; i < errors.cols(); ++i) {
            residual[i * 2]     = errors(0, i);
            residual[i * 2 + 1] = errors(1, i);
        }

        return true;
    }

    Eigen::Matrix3Xd model_points_at_k_;
    Eigen::Matrix2Xd observed_points_at_k_;
    CameraParameterPack camera_parameters_;
    PinholeCamera camera_;
};


// ---------------------------------------------------------------------------
// IMU consistency error between consecutive keyframes k and k+1.
// Integrates IMU observations forward from state k; residual is the
// difference between the integrated prediction and the parameterized state
// at k+1.
//
// Parameters:
//   params_T_world_imu_k    [6]  — IMU pose at k
//   params_v_world_imu_k    [3]  — velocity in world at k
//   params_bias_gyro_k      [3]  — gyro bias at k
//   params_bias_accel_k     [3]  — accel bias at k
//   params_T_world_imu_kp1  [6]  — IMU pose at k+1
//   params_v_world_imu_kp1  [3]  — velocity in world at k+1
//   params_gravity_world    [3]  — gravity vector in world frame
// Residual size: 9  (position[3] + velocity[3] + rotation_log[3])
// ---------------------------------------------------------------------------
class IMUConsistencyError {
public:
    IMUConsistencyError(std::vector<IMUObservation> imu_observations,
                        double time_k_secs);

    static ceres::CostFunction *Create(std::vector<IMUObservation> imu_observations,
                                       double time_k_secs);

    template<typename T>
    bool operator()(
            const T *const params_T_world_imu_k,
            const T *const params_v_world_imu_k,
            const T *const params_bias_gyro_k,
            const T *const params_bias_accel_k,

            const T *const params_T_world_imu_kp1,
            const T *const params_v_world_imu_kp1,

            const T *const params_gravity_world,
            T *residual) const {

        EigenJetAffine<T> T_world_imu_integrated = SE3Transform::toEigenAffineJetSafe(params_T_world_imu_k);
        EigenJetAffine<T> T_world_imu_kp1        = SE3Transform::toEigenAffineJetSafe(params_T_world_imu_kp1);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> v0(params_v_world_imu_k);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bias_accel(params_bias_accel_k);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bias_gyro(params_bias_gyro_k);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> gravity_world(params_gravity_world);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> v_kp1(params_v_world_imu_kp1);

        Eigen::Matrix<T, 3, 1> v = v0;
        double time_i_secs = time_k_secs_;

        for (const auto &obs : imu_observations_k_to_kp1_) {
            const double dt_d = obs.detection_time_secs - time_i_secs;
            if (dt_d < 0) continue;
            time_i_secs = obs.detection_time_secs;

            const T dt  = T(dt_d);
            const T dt2 = dt * dt;

            Eigen::Matrix<T, 3, 1> a = obs.linear_acceleration.cast<T>() - bias_accel;
            Eigen::Matrix<T, 3, 1> w = obs.angular_velocity.cast<T>()    - bias_gyro;

            Eigen::Matrix<T, 3, 3> R = T_world_imu_integrated.linear();
            Eigen::Matrix<T, 3, 1> v_old = v;

            T_world_imu_integrated.translation() +=
                    v_old * dt + T(0.5) * gravity_world * dt2 + T(0.5) * R * a * dt2;

            v = v_old + gravity_world * dt + R * a * dt;

            T_world_imu_integrated.linear() = R * Sophus::SO3<T>::exp(w * dt).matrix();
        }

        // Position residual
        Eigen::Map<Eigen::Matrix<T, 3, 1>> r_position(residual);
        r_position = T_world_imu_integrated.translation() - T_world_imu_kp1.translation();

        // Velocity residual
        Eigen::Map<Eigen::Matrix<T, 3, 1>> r_velocity(residual + 3);
        r_velocity = v - v_kp1;

        // Rotation residual: skew-symmetric part of (R_integrated^T * R_kp1 - I).
        // Equals 2*sin(theta)*axis — a first-order approximation of the rotation vector.
        Eigen::Map<Eigen::Matrix<T, 3, 1>> r_rotation(residual + 6);
        Eigen::Matrix<T, 3, 3> R_err =
                T_world_imu_integrated.linear().transpose() * T_world_imu_kp1.linear();
        r_rotation[0] = R_err(2, 1) - R_err(1, 2);
        r_rotation[1] = R_err(0, 2) - R_err(2, 0);
        r_rotation[2] = R_err(1, 0) - R_err(0, 1);

        return true;
    }

    std::vector<IMUObservation> imu_observations_k_to_kp1_;
    double time_k_secs_;
};


#endif //GRAND_TOUR_CERES_APPS_IMURESIDUALS_H
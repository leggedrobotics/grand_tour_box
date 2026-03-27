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
// Relative IMU board-point residual between consecutive keyframes k and k+1.
//
// The board is the reference (world) frame — it is static and its pose is
// identity by definition. IMU poses are rolled out directly in the board frame.
//
// T_board_imu_k is derived purely from camera observations + extrinsic:
//   T_board_imu_k = inv(T_bundle_board_k) * T_bundle_imu
// where T_bundle_board_k = T_bundle_sensor * T_camera_board_k is baked in.
//
// Gravity is parameterized as a unit direction vector in the board frame,
// scaled by kGravity (9.81 m/s^2) inside the residual.
//
//   p_pred = T_bundle_imu * T_board_imu_{k+1}^{-1} * T_board_imu_k * T_imu_bundle * p_bundle_k
//   residual = p_pred - p_bundle_{k+1}    (R^3 per model point)
//
// Parameters:
//   params_gravity_dir_board   [3]  — unit gravity direction in board frame (global)
//   params_v_board_imu_k       [3]  — IMU velocity in board frame at k (per-keyframe)
//   params_bias_gyro           [3]  — gyroscope bias (global)
//   params_bias_accel          [3]  — accelerometer bias (global)
//   params_T_camera_bundle_imu [7]  — extrinsic: IMU to bundle (global)
// Residual size: 3 * n_model_points
// ---------------------------------------------------------------------------
class RelativeIMUBoardPointError {
public:
    static constexpr double kGravity = 9.81;

    RelativeIMUBoardPointError(std::vector<IMUObservation> imu_data_k_to_kp1,
                               double time_k_secs,
                               double time_kp1_secs,
                               Eigen::Affine3d T_bundle_board_k,
                               Eigen::Matrix3Xd points_in_bundle_k,
                               Eigen::Matrix3Xd points_in_bundle_kp1)
            : imu_data_(std::move(imu_data_k_to_kp1)),
              time_k_secs_(time_k_secs),
              time_kp1_secs_(time_kp1_secs),
              T_bundle_board_k_(std::move(T_bundle_board_k)),
              points_in_bundle_k_(std::move(points_in_bundle_k)),
              points_in_bundle_kp1_(std::move(points_in_bundle_kp1)) {}

    static ceres::CostFunction *Create(std::vector<IMUObservation> imu_data_k_to_kp1,
                                       double time_k_secs,
                                       double time_kp1_secs,
                                       const Eigen::Affine3d &T_bundle_board_k,
                                       const Eigen::Matrix3Xd &points_in_bundle_k,
                                       const Eigen::Matrix3Xd &points_in_bundle_kp1) {
        const int n_residuals = static_cast<int>(points_in_bundle_k.cols()) * 3;
        return new ceres::AutoDiffCostFunction<RelativeIMUBoardPointError, ceres::DYNAMIC,
                3,                              // gravity_dir_board
                3,                              // v_board_imu_k
                3,                              // bias_gyro
                3,                              // bias_accel
                SE3Transform::NUM_PARAMETERS>(  // T_camera_bundle_imu
                new RelativeIMUBoardPointError(std::move(imu_data_k_to_kp1), time_k_secs, time_kp1_secs,
                                               T_bundle_board_k, points_in_bundle_k, points_in_bundle_kp1),
                n_residuals);
    }

    template<typename T>
    bool operator()(
            const T *const params_gravity_dir_board,
            const T *const params_v_board_imu_k,
            const T *const params_bias_gyro,
            const T *const params_bias_accel,
            const T *const params_T_camera_bundle_imu,
            T *residual) const {

        EigenJetAffine<T> T_bundle_imu = SE3Transform::toEigenAffineJetSafe(params_T_camera_bundle_imu);

        // Board frame is the reference: T_board_imu_k = inv(T_bundle_board_k) * T_bundle_imu
        EigenJetAffine<T> T_board_imu = T_bundle_board_k_.cast<T>().inverse() * T_bundle_imu;

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> v0(params_v_board_imu_k);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bias_gyro(params_bias_gyro);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bias_accel(params_bias_accel);

        // Gravity in board frame: unit direction * magnitude
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> g_dir(params_gravity_dir_board);
        Eigen::Matrix<T, 3, 1> g = g_dir * T(kGravity);

        Eigen::Matrix<T, 3, 1> v = v0;
        double time_i = time_k_secs_;

        const size_t n = imu_data_.size();
        for (size_t i = 0; i < n; ++i) {
            const auto &obs   = imu_data_[i];
            const bool is_last = (i + 1 == n);

            const double t_end = is_last ? time_kp1_secs_ : obs.detection_time_secs;
            const double dt_d  = t_end - time_i;
            if (dt_d < 0) { time_i = obs.detection_time_secs; continue; }

            Eigen::Matrix<T, 3, 1> a_meas, w_meas;
            if (is_last && i > 0) {
                const auto &prev  = imu_data_[i - 1];
                const double span = obs.detection_time_secs - prev.detection_time_secs;
                const T alpha     = span > 1e-12 ? T((time_kp1_secs_ - prev.detection_time_secs) / span)
                                                 : T(1);
                a_meas = prev.linear_acceleration.cast<T>() +
                         alpha * (obs.linear_acceleration.cast<T>() - prev.linear_acceleration.cast<T>());
                w_meas = prev.angular_velocity.cast<T>() +
                         alpha * (obs.angular_velocity.cast<T>() - prev.angular_velocity.cast<T>());
            } else {
                a_meas = obs.linear_acceleration.cast<T>();
                w_meas = obs.angular_velocity.cast<T>();
            }

            const T dt  = T(dt_d);
            const T dt2 = dt * dt;
            Eigen::Matrix<T, 3, 1> a      = a_meas - bias_accel;
            Eigen::Matrix<T, 3, 1> w      = w_meas - bias_gyro;
            Eigen::Matrix<T, 3, 3> R      = T_board_imu.linear();
            Eigen::Matrix<T, 3, 1> v_prev = v;

            T_board_imu.translation() += v_prev * dt + T(0.5) * g * dt2 + T(0.5) * R * a * dt2;
            v = v_prev + g * dt + R * a * dt;
            T_board_imu.linear() = R * Sophus::SO3<T>::exp(w * dt).matrix();

            time_i = t_end;
        }

        // T_board_imu is now T_board_imu_{k+1} (integrated).
        // T_board_imu_k: initial IMU pose in board frame at k.
        EigenJetAffine<T> T_board_imu_k = T_bundle_board_k_.cast<T>().inverse() * T_bundle_imu;

        // Predicted bundle-frame points at k+1:
        //   p_pred = T_bundle_imu * T_board_imu_{k+1}^{-1} * T_board_imu_k * T_imu_bundle * p_bundle_k
        Eigen::Matrix<T, 3, Eigen::Dynamic> p_pred =
                T_bundle_imu *
                T_board_imu.inverse() *
                T_board_imu_k *
                T_bundle_imu.inverse() *
                points_in_bundle_k_.cast<T>();

        Eigen::Matrix<T, 3, Eigen::Dynamic> errors =
                p_pred - points_in_bundle_kp1_.cast<T>();

        for (int i = 0; i < errors.cols(); ++i) {
            residual[i * 3]     = errors(0, i);
            residual[i * 3 + 1] = errors(1, i);
            residual[i * 3 + 2] = errors(2, i);
        }
        return true;
    }

private:
    std::vector<IMUObservation> imu_data_;
    double time_k_secs_;
    double time_kp1_secs_;
    Eigen::Affine3d T_bundle_board_k_;
    Eigen::Matrix3Xd points_in_bundle_k_;
    Eigen::Matrix3Xd points_in_bundle_kp1_;
};

// ---------------------------------------------------------------------------
// Tightly-coupled rolled-out residual.
//
// Integrates the IMU from t_0 to t_k using the initial state and global bias
// as parameters — no per-keyframe state variables. The resulting IMU pose is
// used directly to predict where board model points land in the world frame.
//
// Parameters:
//   params_T_world_imu_0       [7]  — initial IMU pose in world
//   params_v_world_imu_0       [3]  — initial velocity in world
//   params_bias_gyro           [3]  — gyroscope bias (global)
//   params_bias_accel          [3]  — accelerometer bias (global)
//   params_T_camera_bundle_imu [7]  — extrinsic: IMU → bundle
//   params_T_world_board       [7]  — board pose in world
// Residual size: 3 * n_model_points
// ---------------------------------------------------------------------------
class IMURolledOutWorldFrameError {
public:
    IMURolledOutWorldFrameError(std::vector<IMUObservation> imu_data_0_to_k,
                                double time_0_secs,
                                double time_k_secs,
                                Eigen::Matrix3Xd points_in_bundle,
                                Eigen::Matrix3Xd model_points,
                                Eigen::Vector3d gravity_world)
            : imu_data_(std::move(imu_data_0_to_k)),
              time_0_secs_(time_0_secs),
              time_k_secs_(time_k_secs),
              points_in_bundle_(std::move(points_in_bundle)),
              model_points_(std::move(model_points)),
              gravity_world_(std::move(gravity_world)) {}

    static ceres::CostFunction *Create(std::vector<IMUObservation> imu_data_0_to_k,
                                       double time_0_secs,
                                       double time_k_secs,
                                       const Eigen::Matrix3Xd &points_in_bundle,
                                       const Eigen::Matrix3Xd &model_points,
                                       const Eigen::Vector3d &gravity_world) {
        const int n_residuals = static_cast<int>(model_points.cols()) * 3;
        return new ceres::AutoDiffCostFunction<IMURolledOutWorldFrameError, ceres::DYNAMIC,
                SE3Transform::NUM_PARAMETERS,   // T_world_imu_0
                3,                              // v_world_imu_0
                3,                              // bias_gyro
                3,                              // bias_accel
                SE3Transform::NUM_PARAMETERS,   // T_camera_bundle_imu
                SE3Transform::NUM_PARAMETERS>(  // T_world_board
                new IMURolledOutWorldFrameError(std::move(imu_data_0_to_k), time_0_secs, time_k_secs,
                                               points_in_bundle, model_points, gravity_world),
                n_residuals);
    }

    template<typename T>
    bool operator()(
            const T *const params_T_world_imu_0,
            const T *const params_v_world_imu_0,
            const T *const params_bias_gyro,
            const T *const params_bias_accel,
            const T *const params_T_camera_bundle_imu,
            const T *const params_T_world_board,
            T *residual) const {

        EigenJetAffine<T> T_world_imu    = SE3Transform::toEigenAffineJetSafe(params_T_world_imu_0);
        EigenJetAffine<T> T_bundle_imu   = SE3Transform::toEigenAffineJetSafe(params_T_camera_bundle_imu);
        EigenJetAffine<T> T_world_board  = SE3Transform::toEigenAffineJetSafe(params_T_world_board);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> v0(params_v_world_imu_0);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bias_gyro(params_bias_gyro);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bias_accel(params_bias_accel);

        Eigen::Matrix<T, 3, 1> g = gravity_world_.cast<T>();
        Eigen::Matrix<T, 3, 1> v = v0;
        double time_i = time_0_secs_;

        const size_t n = imu_data_.size();
        for (size_t i = 0; i < n; ++i) {
            const auto &obs = imu_data_[i];
            const bool is_last = (i + 1 == n);

            const double t_end = is_last ? time_k_secs_ : obs.detection_time_secs;
            const double dt_d  = t_end - time_i;
            if (dt_d < 0) { time_i = obs.detection_time_secs; continue; }

            Eigen::Matrix<T, 3, 1> a_meas, w_meas;
            if (is_last && i > 0) {
                const auto &prev  = imu_data_[i - 1];
                const double span = obs.detection_time_secs - prev.detection_time_secs;
                const T alpha     = span > 1e-12 ? T((time_k_secs_ - prev.detection_time_secs) / span)
                                                 : T(1);
                a_meas = prev.linear_acceleration.cast<T>() +
                         alpha * (obs.linear_acceleration.cast<T>() - prev.linear_acceleration.cast<T>());
                w_meas = prev.angular_velocity.cast<T>() +
                         alpha * (obs.angular_velocity.cast<T>() - prev.angular_velocity.cast<T>());
            } else {
                a_meas = obs.linear_acceleration.cast<T>();
                w_meas = obs.angular_velocity.cast<T>();
            }

            const T dt  = T(dt_d);
            const T dt2 = dt * dt;
            Eigen::Matrix<T, 3, 1> a   = a_meas - bias_accel;
            Eigen::Matrix<T, 3, 1> w   = w_meas - bias_gyro;
            Eigen::Matrix<T, 3, 3> R   = T_world_imu.linear();
            Eigen::Matrix<T, 3, 1> v_prev = v;

            T_world_imu.translation() += v_prev * dt + T(0.5) * g * dt2 + T(0.5) * R * a * dt2;
            v = v_prev + g * dt + R * a * dt;
            T_world_imu.linear() = R * Sophus::SO3<T>::exp(w * dt).matrix();

            time_i = t_end;
        }

        // World-frame 3D point consistency
        Eigen::Matrix<T, 3, Eigen::Dynamic> p_world_imu =
                T_world_imu * T_bundle_imu.inverse() * points_in_bundle_.cast<T>();
        Eigen::Matrix<T, 3, Eigen::Dynamic> p_world_board =
                T_world_board * model_points_.cast<T>();

        Eigen::Matrix<T, 3, Eigen::Dynamic> errors = p_world_imu - p_world_board;
        for (int i = 0; i < errors.cols(); ++i) {
            residual[i * 3]     = errors(0, i);
            residual[i * 3 + 1] = errors(1, i);
            residual[i * 3 + 2] = errors(2, i);
        }
        return true;
    }

private:
    std::vector<IMUObservation> imu_data_;
    double time_0_secs_;
    double time_k_secs_;
    Eigen::Matrix3Xd points_in_bundle_;
    Eigen::Matrix3Xd model_points_;
    Eigen::Vector3d gravity_world_;
};

// ---------------------------------------------------------------------------
// World-frame 3D point consistency error at a single keyframe k.
//
// Instead of projecting through the camera, this residual works entirely in
// world coordinates. T_camera_board (from solvePnP) and T_bundle_sensor are
// baked in at construction; points_in_bundle is precomputed once.
//
// Residual: T_world_imu_k * inv(T_bundle_imu) * p_bundle - T_world_board * p_model
//           (3 residuals per model point, no projection non-linearity)
//
// Parameters:
//   params_T_camera_bundle_imu  [7]  — extrinsic: IMU → bundle
//   params_T_world_board        [7]  — board pose in world
//   params_T_world_imu_k        [7]  — IMU pose at keyframe k
// ---------------------------------------------------------------------------
class WorldFramePointConsistencyError {
public:
    // points_in_bundle  = T_bundle_sensor * T_camera_board * model_points  (precomputed)
    // model_points      = 3D board model points
    WorldFramePointConsistencyError(Eigen::Matrix3Xd points_in_bundle,
                                    Eigen::Matrix3Xd model_points)
            : points_in_bundle_(std::move(points_in_bundle)),
              model_points_(std::move(model_points)) {}

    static ceres::CostFunction *Create(const Eigen::Matrix3Xd &points_in_bundle,
                                       const Eigen::Matrix3Xd &model_points) {
        const int n_residuals = static_cast<int>(model_points.cols()) * 3;
        return new ceres::AutoDiffCostFunction<WorldFramePointConsistencyError, ceres::DYNAMIC,
                SE3Transform::NUM_PARAMETERS,   // T_camera_bundle_imu
                SE3Transform::NUM_PARAMETERS,   // T_world_board
                SE3Transform::NUM_PARAMETERS>(  // T_world_imu_k
                new WorldFramePointConsistencyError(points_in_bundle, model_points),
                n_residuals);
    }

    template<typename T>
    bool operator()(
            const T *const params_T_camera_bundle_imu,
            const T *const params_T_world_board,
            const T *const params_T_world_imu_k,
            T *residual) const {

        EigenJetAffine<T> T_bundle_imu  = SE3Transform::toEigenAffineJetSafe(params_T_camera_bundle_imu);
        EigenJetAffine<T> T_world_board = SE3Transform::toEigenAffineJetSafe(params_T_world_board);
        EigenJetAffine<T> T_world_imu_k = SE3Transform::toEigenAffineJetSafe(params_T_world_imu_k);

        // IMU-chain prediction: roll the bundle-frame points into world via IMU pose
        Eigen::Matrix<T, 3, Eigen::Dynamic> p_world_imu =
                T_world_imu_k * T_bundle_imu.inverse() * points_in_bundle_.cast<T>();

        // Board prediction: transform model points into world via board pose
        Eigen::Matrix<T, 3, Eigen::Dynamic> p_world_board =
                T_world_board * model_points_.cast<T>();

        Eigen::Matrix<T, 3, Eigen::Dynamic> errors = p_world_imu - p_world_board;
        for (int i = 0; i < errors.cols(); ++i) {
            residual[i * 3]     = errors(0, i);
            residual[i * 3 + 1] = errors(1, i);
            residual[i * 3 + 2] = errors(2, i);
        }
        return true;
    }

private:
    Eigen::Matrix3Xd points_in_bundle_;  // T_bundle_sensor * T_camera_board * model_pts
    Eigen::Matrix3Xd model_points_;
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
                        double time_k_secs,
                        double time_kp1_secs);

    static ceres::CostFunction *Create(std::vector<IMUObservation> imu_observations,
                                       double time_k_secs,
                                       double time_kp1_secs);

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

        const size_t n = imu_observations_k_to_kp1_.size();
        for (size_t i = 0; i < n; ++i) {
            const auto &obs = imu_observations_k_to_kp1_[i];
            const bool is_last = (i + 1 == n);

            // For the final sample (first one >= t_kp1): integrate only to t_kp1
            // and linearly interpolate the measurement between prev and this sample.
            const double t_end = is_last ? time_kp1_secs_ : obs.detection_time_secs;
            const double dt_d = t_end - time_i_secs;
            if (dt_d < 0) { time_i_secs = obs.detection_time_secs; continue; }

            Eigen::Matrix<T, 3, 1> a_meas, w_meas;
            if (is_last && i > 0) {
                const auto &prev = imu_observations_k_to_kp1_[i - 1];
                const double span = obs.detection_time_secs - prev.detection_time_secs;
                const T alpha = span > 1e-12 ? T((time_kp1_secs_ - prev.detection_time_secs) / span)
                                             : T(1);
                a_meas = prev.linear_acceleration.cast<T>() +
                         alpha * (obs.linear_acceleration.cast<T>() - prev.linear_acceleration.cast<T>());
                w_meas = prev.angular_velocity.cast<T>() +
                         alpha * (obs.angular_velocity.cast<T>() - prev.angular_velocity.cast<T>());
            } else {
                a_meas = obs.linear_acceleration.cast<T>();
                w_meas = obs.angular_velocity.cast<T>();
            }

            const T dt  = T(dt_d);
            const T dt2 = dt * dt;

            Eigen::Matrix<T, 3, 1> a = a_meas - bias_accel;
            Eigen::Matrix<T, 3, 1> w = w_meas - bias_gyro;

            Eigen::Matrix<T, 3, 3> R = T_world_imu_integrated.linear();
            Eigen::Matrix<T, 3, 1> v_old = v;

            T_world_imu_integrated.translation() +=
                    v_old * dt + T(0.5) * gravity_world * dt2 + T(0.5) * R * a * dt2;

            v = v_old + gravity_world * dt + R * a * dt;

            T_world_imu_integrated.linear() = R * Sophus::SO3<T>::exp(w * dt).matrix();

            time_i_secs = t_end;
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
    double time_kp1_secs_;
};


#endif //GRAND_TOUR_CERES_APPS_IMURESIDUALS_H
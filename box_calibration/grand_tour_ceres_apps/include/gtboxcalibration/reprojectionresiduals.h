//
// Created by fu on 08/08/2024.
//

#ifndef GRAND_TOUR_CERES_APPS_REPROJECTION_ERRORS_H
#define GRAND_TOUR_CERES_APPS_REPROJECTION_ERRORS_H

#include <gtboxcalibration/camerageometry.h>
#include <Eigen/Dense>
#include <utility>


class TransformDistortPinholeProjectError {
public:

    TransformDistortPinholeProjectError(PinholeCamera camera,
                                        Eigen::Matrix3Xd modelPoints,
                                        Eigen::Matrix2Xd observedPoints2D);

    template<typename T>
    bool operator()(
            const T *const params_T_cam_board,
            const T *const params_distortion,
            const T *const params_fxfycxcy,
            T *residual) const {

        Eigen::Matrix<T, 3, Eigen::Dynamic> model_points_t = model_points_.cast<T>();

        Eigen::Matrix<T, 2, Eigen::Dynamic> projected_coordinates = camera_.template transformToCameraThenProject(
                params_T_cam_board, params_fxfycxcy, params_distortion, model_points_t
        );

        Eigen::Matrix<T, 2, Eigen::Dynamic> errors = observed_points2d_ - projected_coordinates;
        for (int i = 0; i < errors.cols(); i++) {
            residual[i * 2] = errors(0, i);
            residual[i * 2 + 1] = errors(1, i);
        }

        return true;
    }

    static ceres::CostFunction *Create(PinholeCamera camera, Eigen::Matrix3Xd modelPoints,
                                       Eigen::Matrix2Xd observedPoints2D,
                                       int num_residuals);

private:
    PinholeCamera camera_;
    Eigen::Matrix3Xd model_points_;
    Eigen::Matrix2Xd observed_points2d_;
};

class StereoCameraReprojectionError {
public:
    StereoCameraReprojectionError(PinholeCamera cameraA, PinholeCamera cameraB,
                                  Eigen::Matrix3Xd modelPoints, Eigen::Matrix2Xd observedPoints2DA,
                                  Eigen::Matrix2Xd observedPoints2DB);

    template<typename T>
    bool operator()(
            const T *const params_T_cam_a_board,
            const T *const params_T_world_cam_a,
            const T *const params_distortion_a,
            const T *const params_fxfycxcy_a,
            const T *const params_T_cam_b_board,
            const T *const params_T_world_cam_b,
            const T *const params_distortion_b,
            const T *const params_fxfycxcy_b,
            T *residual) const {

        Eigen::Matrix<T, 3, Eigen::Dynamic> model_points_t = model_points_.cast<T>();

        Eigen::Matrix<T, 2, Eigen::Dynamic> points_sensorA_in_cameraB =
                camera_b.template project(params_fxfycxcy_b, params_distortion_b,
                                          se3_transform.template inverseTransform(params_T_world_cam_b,
                                                                                  se3_transform(params_T_world_cam_a,
                                                                                                se3_transform(
                                                                                                        params_T_cam_a_board,
                                                                                                        model_points_t))));

        Eigen::Matrix<T, 2, Eigen::Dynamic> points_sensorB_in_cameraA =
                camera_a.template project(params_fxfycxcy_a, params_distortion_a,
                                          se3_transform.template inverseTransform(params_T_world_cam_a,
                                                                                  se3_transform(params_T_world_cam_b,
                                                                                                se3_transform(
                                                                                                        params_T_cam_b_board,
                                                                                                        model_points_t))));
        Eigen::Matrix<T, 2, Eigen::Dynamic> reprojection_errors_a =
                observed_points2d_a - points_sensorB_in_cameraA;
        Eigen::Matrix<T, 2, Eigen::Dynamic> reprojection_errors_b =
                observed_points2d_b - points_sensorA_in_cameraB;


        for (int i = 0; i < reprojection_errors_a.cols(); i++) {
            residual[i * 2] = reprojection_errors_a(0, i);
            residual[i * 2 + 1] = reprojection_errors_a(1, i);
        }

        const int num_a_residuals = reprojection_errors_a.cols() * 2;

        for (int i = 0; i < reprojection_errors_b.cols(); i++) {
            residual[num_a_residuals + i * 2] = reprojection_errors_b(0, i);
            residual[num_a_residuals + i * 2 + 1] = reprojection_errors_b(1, i);
        }

        return true;
    }


    static ceres::CostFunction *Create(PinholeCamera cameraA, PinholeCamera cameraB,
                                       Eigen::Matrix3Xd modelPoints, Eigen::Matrix2Xd observedPoints2DA,
                                       Eigen::Matrix2Xd observedPoints2DB);

private:
    PinholeCamera camera_a;
    PinholeCamera camera_b;
    Eigen::Matrix3Xd model_points_;
    Eigen::Matrix2Xd observed_points2d_a;
    Eigen::Matrix2Xd observed_points2d_b;
    SE3Transform se3_transform;
};

class Model3DSensorAWorldModel3DSensorBWorldConsistencyError {
public:

    explicit Model3DSensorAWorldModel3DSensorBWorldConsistencyError(Eigen::Matrix3Xd modelPoints);

    template<typename T>
    bool operator()(
            const T *const params_T_sensora_model,
            const T *const params_T_bundle_sensora,
            const T *const params_T_sensorb_model,
            const T *const params_T_bundle_sensorb,
            T *residual) const {

        Eigen::Matrix<T, 3, Eigen::Dynamic> model_points_t = model_points_.cast<T>();

        Eigen::Matrix<T, 3, Eigen::Dynamic> points_sensor_a_in_world =
                se3_transform(params_T_bundle_sensora,
                              se3_transform(params_T_sensora_model, model_points_t));

        Eigen::Matrix<T, 3, Eigen::Dynamic> points_sensor_b_in_world =
                se3_transform(params_T_bundle_sensorb,
                              se3_transform(params_T_sensorb_model, model_points_t));

        Eigen::Matrix<T, 3, Eigen::Dynamic> errors = points_sensor_a_in_world - points_sensor_b_in_world;
        for (int i = 0; i < errors.cols(); i++) {
            residual[i * 3] = errors(0, i);
            residual[i * 3 + 1] = errors(1, i);
            residual[i * 3 + 2] = errors(2, i);
        }
        return true;
    }

    static ceres::CostFunction *Create(
            Eigen::Matrix3Xd modelPoints,
            int num_residuals) {
        return new ceres::AutoDiffCostFunction<Model3DSensorAWorldModel3DSensorBWorldConsistencyError, ceres::DYNAMIC,
                SE3Transform::NUM_PARAMETERS,
                SE3Transform::NUM_PARAMETERS,
                SE3Transform::NUM_PARAMETERS,
                SE3Transform::NUM_PARAMETERS>(
                new Model3DSensorAWorldModel3DSensorBWorldConsistencyError(std::move(modelPoints)),
                num_residuals
        );
    }

private:
    SE3Transform se3_transform;
    Eigen::Matrix3Xd model_points_;
};

#endif //GRAND_TOUR_CERES_APPS_REPROJECTION_ERRORS_H

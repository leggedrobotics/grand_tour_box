//
// Created by fu on 05/09/2024.
//

#include <gtboxcalibration/reprojectionresiduals.h>

TransformDistortPinholeProjectError::TransformDistortPinholeProjectError(PinholeCamera camera,
                                                                         Eigen::Matrix3Xd modelPoints,
                                                                         Eigen::Matrix2Xd observedPoints2D) :
        camera_(std::move(camera)),
        model_points_(std::move(modelPoints)),
        observed_points2d_(std::move(
                observedPoints2D)) {}

ceres::CostFunction *TransformDistortPinholeProjectError::Create(PinholeCamera camera, Eigen::Matrix3Xd modelPoints,
                                                                 Eigen::Matrix2Xd observedPoints2D, int num_residuals) {
    return new ceres::AutoDiffCostFunction<TransformDistortPinholeProjectError, ceres::DYNAMIC,
            SE3Transform::NUM_PARAMETERS,
            Distortion::NUM_PARAMETERS,
            PinholeProjection::NUM_PARAMETERS>(
            new TransformDistortPinholeProjectError(std::move(camera), std::move(modelPoints),
                                                    std::move(observedPoints2D)),
            num_residuals
    );
}

StereoCameraReprojectionError::StereoCameraReprojectionError(PinholeCamera cameraA, PinholeCamera cameraB,
                                                             Eigen::Matrix3Xd modelPoints,
                                                             Eigen::Matrix2Xd observedPoints2DA,
                                                             Eigen::Matrix2Xd observedPoints2DB)
        : camera_a(std::move(cameraA)), camera_b(std::move(cameraB)),
          model_points_(std::move(modelPoints)), observed_points2d_a(std::move(observedPoints2DA)),
          observed_points2d_b(std::move(observedPoints2DB)) {}

ceres::CostFunction *
StereoCameraReprojectionError::Create(PinholeCamera cameraA, PinholeCamera cameraB, Eigen::Matrix3Xd modelPoints,
                                      Eigen::Matrix2Xd observedPoints2DA, Eigen::Matrix2Xd observedPoints2DB) {
    int n_residuals = modelPoints.cols() * 2 * 2;
    return new ceres::AutoDiffCostFunction<StereoCameraReprojectionError, ceres::DYNAMIC,
            SE3Transform::NUM_PARAMETERS,
            SE3Transform::NUM_PARAMETERS,
            Distortion::NUM_PARAMETERS,
            PinholeProjection::NUM_PARAMETERS,
            SE3Transform::NUM_PARAMETERS,
            SE3Transform::NUM_PARAMETERS,
            Distortion::NUM_PARAMETERS,
            PinholeProjection::NUM_PARAMETERS>(
            new StereoCameraReprojectionError(std::move(cameraA), std::move(cameraB), std::move(modelPoints),
                                              std::move(observedPoints2DA), std::move(observedPoints2DB)),
            n_residuals
    );
}

Model3DSensorAWorldModel3DSensorBWorldConsistencyError::Model3DSensorAWorldModel3DSensorBWorldConsistencyError(
        Eigen::Matrix3Xd modelPoints) :
        model_points_(std::move(modelPoints)) {}

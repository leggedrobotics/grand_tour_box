//
// Created by fu on 05/09/2024.
//

#include <map>
#include <string>
#include <gtboxcalibration/parameterhelpers.h>
#include <Eigen/Dense>
#include "gtboxcalibration/callbacks.h"

CameraParameterPackIterationCallback::CameraParameterPackIterationCallback(
        const std::map<std::string, CameraParameterPack> *parametersPack, const rerun::RecordingStream *rec)
        : parameters_pack(parametersPack), rec(rec) {}

ceres::CallbackReturnType CameraParameterPackIterationCallback::operator()(const ceres::IterationSummary &summary) {
    rec->set_time_seconds("stable_time", summary.iteration);
    for (const auto&[name, params]: *parameters_pack) {
        SE3Transform se3Transform;
        Eigen::Matrix4f T = se3Transform.toEigen(params.T_bundle_sensor).cast<float>();
        Eigen::Vector3f translation = T.block<3, 1>(0, 3);
        Eigen::Matrix3f rotation = T.block<3, 3>(0, 0);
        rec->log(
                "world/" + name,
                rerun::Transform3D(
                        rerun::Vec3D(translation.data()),
                        rerun::Mat3x3(rotation.data())
                )
        );
    }
    return ceres::SOLVER_CONTINUE;
}

BoardPoseParameterPackCallback::BoardPoseParameterPackCallback(
        const std::map<std::string, std::map<unsigned long long, BoardPoseParameterPack>> *parametersPack,
        const rerun::RecordingStream *rec) : parameters_pack(parametersPack), rec(rec) {}

ceres::CallbackReturnType BoardPoseParameterPackCallback::operator()(const ceres::IterationSummary &summary) {
    for (const auto&[name, pack]: *parameters_pack) {
        for (const auto&[stamp, params]: pack) {
            rec->set_time_nanos("box_time", stamp);
            SE3Transform se3Transform;
            Eigen::Matrix4f T = se3Transform.toEigen(params.T_sensor_board).cast<float>();
            Eigen::Vector3f translation = T.block<3, 1>(0, 3);
            Eigen::Matrix3f rotation = T.block<3, 3>(0, 0);
            rec->log(
                    "world/iteration_number_" + std::to_string(summary.iteration) + "/" + name + "/board_pose",
                    rerun::Transform3D(
                            rerun::Vec3D(translation.data()),
                            rerun::Mat3x3(rotation.data())
                    )
            );
        }
    }
    return ceres::SOLVER_CONTINUE;
}

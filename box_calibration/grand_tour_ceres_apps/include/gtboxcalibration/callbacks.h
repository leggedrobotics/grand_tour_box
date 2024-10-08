//
// Created by fu on 26/08/2024.
//

#ifndef GRAND_TOUR_CERES_APPS_ITERATION_CALLBACKS_H
#define GRAND_TOUR_CERES_APPS_ITERATION_CALLBACKS_H

#include <rerun.hpp>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <gtboxcalibration/parameterhelpers.h>

class CameraParameterPackIterationCallback : public ceres::IterationCallback {
public:
    CameraParameterPackIterationCallback(const std::map<std::string, CameraParameterPack> *parametersPack,
                                         const rerun::RecordingStream *rec);

private:
    const std::map<std::string, CameraParameterPack> *parameters_pack;
    const rerun::RecordingStream *rec;
public:
    ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override;

};

class BoardPoseParameterPackCallback : public ceres::IterationCallback {
public:
    BoardPoseParameterPackCallback(
            const std::map<std::string, std::map<unsigned long long, BoardPoseParameterPack>> *parametersPack,
            const rerun::RecordingStream *rec);

private:
    const std::map<std::string, std::map<unsigned long long, BoardPoseParameterPack>> *parameters_pack;
    const rerun::RecordingStream *rec;
public:
    ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override;

};


#endif //GRAND_TOUR_CERES_APPS_ITERATION_CALLBACKS_H

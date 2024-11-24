#pragma once

#include <Eigen/Geometry>
#include <map>
#include "ros_camera_prism_parser.h"
#include "gtboxcalibration/ceresprograms.h"
#include <gtboxcalibration/parameterhelpers.h>
#include <ros/time.h>

struct ROSCameraPrismProgram : public CameraPrismProgram {
    explicit ROSCameraPrismProgram(ROSCameraPrismParser parser);

    bool Solve() override;

    std::map<std::string, CameraParameterPack> camera_packs;
};
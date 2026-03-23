#pragma once

#include <Eigen/Geometry>
#include <map>
#include "ros_camera_prism_parser.h"
#include "gtboxcalibration/ceresprograms.h"
#include "camera_prism_viz_interface.h"
#include <gtboxcalibration/parameterhelpers.h>
#include <ros/time.h>

struct ROSCameraPrismProgram : public CameraPrismProgram {
    explicit ROSCameraPrismProgram(ROSCameraPrismParser parser,
                                   std::unique_ptr<CameraPrismVizInterface> camera_camera_program);

    bool Solve() override;

    void calculateResiduals();

    void visualizePerCameraPrismEstimates();

    std::map<std::string, CameraParameterPack> camera_packs;
    std::unique_ptr<CameraPrismVizInterface> viz_;
};
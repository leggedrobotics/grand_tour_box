#pragma once

#include <Eigen/Geometry>
#include <map>
#include "ros_camera_prism_parser.h"
#include "gtboxcalibration/ceresprograms.h"
#include "camera_imu_viz_interface.h"
#include "ros_camera_imu_parser.h"
#include <gtboxcalibration/parameterhelpers.h>
#include <ros/time.h>
#include <gtboxcalibration/imuresiduals.h>

struct ROSCameraIMUProgram : public CameraIMUProgram {
    explicit ROSCameraIMUProgram(ROSCameraIMUParser parser,
                                 std::unique_ptr<CameraImuVizInterface> camera_camera_program);

    bool Solve() override;



    std::unique_ptr<CameraImuVizInterface> viz_;
};
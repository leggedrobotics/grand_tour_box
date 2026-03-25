#pragma once

#include <Eigen/Geometry>
#include <map>
#include "ros_camera_prism_parser.h"
#include "gtboxcalibration/ceresprograms.h"
#include "camera_imu_viz_interface.h"
#include "ros_camera_imu_parser.h"
#include <gtboxcalibration/parameterhelpers.h>
#include <ros/time.h>

struct ROSCameraIMUProgram : public CameraPrismProgram {
    explicit ROSCameraIMUProgram(ROSCameraIMUParser parser,
                                 std::unique_ptr<CameraImuVizInterface> camera_camera_program);

    bool Solve() override;

    void calculateResidualsAndFilterOutliers();

    std::map<std::string, CameraParameterPack> camera_packs;
    std::unique_ptr<CameraImuVizInterface> viz_;

    std::map<unsigned long long int, std::map<std::string, double>>
    computeResidualNormMap(std::vector<double> &all_residuals);

    std::map<std::string, std::vector<Eigen::Vector3d>> computeResidualMap3D();
};
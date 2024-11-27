//
// Created by fu on 07/08/2024.
//

#ifndef GRAND_TOUR_CERES_APPS_UTILS_H
#define GRAND_TOUR_CERES_APPS_UTILS_H

#include <filesystem>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <fstream>
#include <gtboxcalibration/parameterhelpers.h>
#include <gtboxcalibration/json.h>
#include <gtboxcalibration/datatypes.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

int dummy_function_for_tests_addition(int a, int b);

std::string GetHumanReadableTime(int64_t calibration_time_nsec);

std::map<std::string, Eigen::Affine3d> FetchExtrinsicsFromYaml(const YAML::Node &extrinsics_data);

std::map<std::string, CameraParameterPack> FetchIntrinsicsAsCameraPackFromYaml(const YAML::Node &kalibr_data);

std::map<std::string, Eigen::Affine3d> FetchExtrinsicsFromYamlPath(std::string yaml_path);

CameraCamera2D3DTargetDetectionData FetchMulticamera2D3DDetectionData(std::string path);

std::map<std::string, CameraParameterPack> PopulateCameraParameterPacks(const std::string &intrinsics_yaml_path,
                                                                        const std::string &g_path);

bool SerialiseCameraParameters(const std::string &output_path,
                               const std::map<std::string, CameraParameterPack> &camera_parameter_packs,
                               const std::string comment = "");

std::map<std::string, std::map<unsigned long long, std::shared_ptr<BoardPoseParameterPack>>>
PopulateBoardParameters(const CameraCamera2D3DTargetDetectionData
                        &parsed_alignment_data);

PrismPositionDetectionData LoadPrismPositions(std::string json_path);

// Function to evaluate and collect all residuals
void GetAllResiduals(ceres::Problem &problem, const std::vector<ceres::ResidualBlockId> &residual_blocks,
                     Eigen::Matrix2Xf &residuals);


#endif //GRAND_TOUR_CERES_APPS_UTILS_H

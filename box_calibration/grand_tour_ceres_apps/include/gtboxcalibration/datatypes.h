//
// Created by fu on 28/08/2024.
//

#ifndef COMPUTE_CONNECTIVITY_TYPES_H
#define COMPUTE_CONNECTIVITY_TYPES_H

#include <Eigen/Dense>
#include <string>
#include <set>
#include <map>

struct Observations2dModelPoints3dPointIDsPose3dSensorName {
    Eigen::Matrix2Xd observations2d;
    Eigen::Matrix3Xd modelpoints3d;
    std::vector<unsigned int>  modelpointIDs;
    Eigen::Affine3d T_sensor_model;
    std::string sensor_name;
};

struct CameraCamera2D3DTargetDetectionData {
    std::set<unsigned long long> unique_timestamps;
    std::map<unsigned long long, std::map<std::string, Observations2dModelPoints3dPointIDsPose3dSensorName>> observations;
};

using PrismPositionDetectionData = std::map<unsigned long long, Eigen::Vector3d>;


#endif //COMPUTE_CONNECTIVITY_TYPES_H

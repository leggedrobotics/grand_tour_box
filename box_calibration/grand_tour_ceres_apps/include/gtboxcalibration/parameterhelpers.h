//
// Created by fu on 12/08/2024.
//

#ifndef GRAND_TOUR_CERES_APPS_PARAMETER_HELPERS_H
#define GRAND_TOUR_CERES_APPS_PARAMETER_HELPERS_H


#include <gtboxcalibration/camerageometry.h>

struct CameraParameterPack {
    double T_bundle_sensor[SE3Transform::NUM_PARAMETERS]{};   // 7
    double fxfycxcy[FisheyeDistortion::NUM_PARAMETERS]{};     // Variable but 4 for now
    double dist_coeffs[4]{};                                  // Fix to 4 parameters for now
    Distortion::Type distortion_type{};
    int width{};
    int height{};
};

struct BoardPoseParameterPack {
    double T_sensor_board[SE3Transform::NUM_PARAMETERS]{0, 0, 0, 0, 0, 0, 0};
};

struct PrismBoardInTotalStationParameterPack {
    double T_totalstation_board[SE3Transform::NUM_PARAMETERS]{};
    double t_cam0_prism[3]{};
    double t_offset[1]{};
};

struct ImuParameterPack {
    double T_camera_bundle_imu[SE3Transform::NUM_PARAMETERS]{};
    double T_world_imu[SE3Transform::NUM_PARAMETERS]{};
    double v_world_imu[3]{};
    double gravity_world[3]{};
    double bias_gyro[3]{};
    double bias_accel[3]{};
    double T_world_board[SE3Transform::NUM_PARAMETERS]{};
};

// Per-keyframe IMU state: explicit SE3 pose + velocity + biases in the board frame.
struct ImuKeyframeParameterPack {
    double T_board_imu[SE3Transform::NUM_PARAMETERS]{0, 0, 0, 1, 0, 0, 0};
    double v_board_imu[3]{};
    double bias_gyro[3]{};
    double bias_accel[3]{};
};

#endif //GRAND_TOUR_CERES_APPS_PARAMETER_HELPERS_H

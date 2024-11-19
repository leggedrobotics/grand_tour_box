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

#endif //GRAND_TOUR_CERES_APPS_PARAMETER_HELPERS_H

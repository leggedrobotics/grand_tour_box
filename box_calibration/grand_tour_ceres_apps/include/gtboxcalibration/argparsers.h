//
// Created by fu on 09/09/2024.
//

#ifndef COMPUTE_CONNECTIVITY_ARGPARSERS_H
#define COMPUTE_CONNECTIVITY_ARGPARSERS_H

#include <gtboxcalibration/argparse.h>

struct CameraCameraCalibrationAppParser {
    CameraCameraCalibrationAppParser(int argc, char *argv[]);

    std::string intrinsics_path;
    std::string initial_guess_path;
    std::string alignment_data_path;
    std::string output_root;
    bool is_valid = false;
};

struct CameraPrismCalibrationAppParser {
    CameraPrismCalibrationAppParser(int argc, char *argv[]);

    virtual ~CameraPrismCalibrationAppParser();

    std::string prism_positions_path;
    std::string intrinsics_path;
    std::string extrinsics_path;
    std::string camera_corner_detections_path;
    bool is_valid_ = false;
};


#endif //COMPUTE_CONNECTIVITY_ARGPARSERS_H

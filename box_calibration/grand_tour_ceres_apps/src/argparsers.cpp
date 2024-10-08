//
// Created by fu on 09/09/2024.
//

#include <gtboxcalibration/argparsers.h>

CameraCameraCalibrationAppParser::CameraCameraCalibrationAppParser(int argc, char **argv) {
    argparse::ArgumentParser program("CameraCameraCalibrationApp");
    program.add_argument("-a", "--alignment_data")
            .required()
            .help("specify the file containing all 2d/3d observations.");
    program.add_argument("-i", "--intrinsics")
            .required()
            .help("specify the file containing kalibr intrinsics.");
    program.add_argument("-t", "--target")
            .required()
            .help("specify the file containing kalibr-style target.");
    program.add_argument("-g", "--extrinsics_guess")
            .required()
            .help("specify the file containing kalibr-style initial guess of extrinsics.");
    program.add_argument("-o", "--output_root")
            .required()
            .help("specify the output root folder.");
    try {
        program.parse_args(argc, argv);
    }
    catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return;
    }
    alignment_data_path = program.get<std::string>("a");
    intrinsics_path = program.get<std::string>("i");
    initial_guess_path = program.get<std::string>("g");
    output_root = program.get<std::string>("o");
    is_valid = true;
}

CameraPrismCalibrationAppParser::CameraPrismCalibrationAppParser(int argc, char **argv) {
    argparse::ArgumentParser program("CameraPrismCalibrationApp");
    program.add_argument("-c", "--camera_corner_detections")
            .required()
            .help("specify the file containing all timestamped camera 2d/3d observations.");
    program.add_argument("-i", "--camera_intrinsics")
            .required()
            .help("specify the file containing intrinsics.");
    program.add_argument("-e", "--camera_intrinsics")
            .required()
            .help("specify the file containing extrinsics.");
    program.add_argument("-p", "--prism_positions")
            .required()
            .help("specify the file containing timestamp, prism point detection.");
    try {
        program.parse_args(argc, argv);
    }
    catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return;
    }
    camera_corner_detections_path = program.get<std::string>("c");
    intrinsics_path = program.get<std::string>("i");
    extrinsics_path = program.get<std::string>("e");
    prism_positions_path = program.get<std::string>("p");

    std::cout << "Running Camera/Prism calibration using:\n"
    << "camera detections file: " << camera_corner_detections_path << "\n"
    << "camera intrinsics file: " << intrinsics_path << "\n"
    << "camera extrinsics file: " << extrinsics_path << "\n"
    << "prism positions file: " << prism_positions_path << "\n";
    is_valid_ = true;
}

CameraPrismCalibrationAppParser::~CameraPrismCalibrationAppParser() {
}

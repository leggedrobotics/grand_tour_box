#include "ros_camera_prism_parser.h"
#include <gtboxcalibration/argparsers.h>

ROSCameraPrismParser::ROSCameraPrismParser(std::string program_name, int argc, char **argv) {
    argparse::ArgumentParser program(program_name);
    program.add_argument("-c", "--cameras_calibration_path")
            .required()
            .help("specify the file containing camera intrinsics and extrinsics.");
    program.add_argument("--camera_bags")
            .help("Paths to rosbags with grand_tour_camera_detection_msgs/CameraDetections topics")
            .nargs(argparse::nargs_pattern::any) // Allows variadic arguments
            .default_value(std::vector<std::string>{}) // Default to an empty vector if none are provided
            .action([](const std::string& value) { return value; });
    program.add_argument("-p", "--prism_bag")
            .required()
            .help("specify the output root folder.");
    program.add_argument("-o", "--output_path")
            .required()
            .help("Output yaml file path");
    program.add_argument("-t", "--prism_topic")
            .required()
            .help("Rostopic of the prism position.");
    program.add_argument("--solve_time_offset")
            .default_value(false)
            .implicit_value(true)
            .help("Boolean flag to solve the timeoffset.");
    try {
        program.parse_args(argc, argv);
    }
    catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return;
    }
    // Retrieve the list of bags
    auto bags = program.get<std::vector<std::string>>("--camera_bags");
    cameras_calibration_path = program.get<std::string>("--cameras_calibration_path");
    camera_bag_paths = bags;
    prism_bag_path = program.get<std::string>("--prism_bag");
    output_path = program.get<std::string>("--output_path");
    prism_topic = program.get<std::string>("--prism_topic");
    solve_time_offset = program.get<bool>("--solve_time_offset");
    is_valid = true;
}

#include "ros_camera_imu_parser.h"
#include <gtboxcalibration/argparsers.h>
#include <ros/package.h>
#include <filesystem>

ROSCameraIMUParser::ROSCameraIMUParser(std::string program_name, int argc, char **argv) {
    argparse::ArgumentParser program(program_name);
    const std::string package_name = "grand_tour_ceres_apps";
    std::string package_path = ros::package::getPath(package_name);
    std::string rerun_blueprint_default_path = package_path + "/config/camera_imu.rbl";
    program.add_argument("-c", "--cameras_calibration_path")
            .required()
            .help("specify the file containing camera intrinsics and extrinsics.");
    program.add_argument("--camera_detection_bags")
            .help("Paths to rosbags with grand_tour_camera_detection_msgs/CameraDetections topics")
            .nargs(argparse::nargs_pattern::any) // Allows variadic arguments
            .default_value(std::vector<std::string>{}) // Default to an empty vector if none are provided
            .action([](const std::string& value) { return value; });
    program.add_argument("-b", "--imu_bag")
            .required()
            .help("Input bag containing IMU measurements.");
    program.add_argument("-o", "--output_path")
            .required()
            .help("Output yaml file path");
    program.add_argument("-t", "--imu_topic")
            .required()
            .help("ROS topic containing sensor_msgs/Imu measurements.");
    program.add_argument("--solve_time_offset")
            .default_value(false)
            .implicit_value(true)
            .help("Boolean flag to solve the timeoffset.");
    program.add_argument("--blueprint_path")
            .help("Path to the rerun blueprint file.")
            .default_value(rerun_blueprint_default_path);
    program.add_argument("--visualize").default_value(false).implicit_value(true)
            .help("Open the Rerun viewer during calibration.");
    program.add_argument("--rerun_recording").default_value(std::string{})
            .help("Save visualization to an .rrd file without opening a viewer.");
    try {
        program.parse_args(argc, argv);
    }
    catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return;
    }
    // Retrieve the list of bags
    auto bags = program.get<std::vector<std::string>>("--camera_detection_bags");
    cameras_calibration_path = program.get<std::string>("--cameras_calibration_path");
    camera_detection_bag_paths = bags;
    imu_bag_path = program.get<std::string>("--imu_bag");
    output_path = program.get<std::string>("--output_path");
    imu_topic = program.get<std::string>("--imu_topic");
    solve_time_offset = program.get<bool>("--solve_time_offset");
    // No residual currently contains a time-offset parameter. Reject the option
    // explicitly instead of silently returning a spatial-only calibration.
    if (solve_time_offset) {
        std::cerr << "Camera IMU time-offset estimation is not implemented.\n";
        return;
    }
    if (camera_detection_bag_paths.empty() ||
        !std::filesystem::is_regular_file(cameras_calibration_path) ||
        !std::filesystem::is_regular_file(imu_bag_path)) return;
    for (const auto& path : camera_detection_bag_paths)
        if (!std::filesystem::is_regular_file(path)) return;
    is_valid = !output_path.empty() && !imu_topic.empty();
    visualize = program.get<bool>("--visualize");
    rerun_recording = program.get<std::string>("--rerun_recording");
    rerun_blue_print_path = program.get<std::string>("--blueprint_path");
}

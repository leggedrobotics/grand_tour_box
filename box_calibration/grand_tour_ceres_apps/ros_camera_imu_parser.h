#pragma once

#include <string>
#include <vector>

struct ROSCameraIMUParser {
    ROSCameraIMUParser(std::string program_name, int argc, char *argv[]);
    std::string cameras_calibration_path;
    std::string output_path;
    std::vector<std::string> camera_detection_bag_paths;
    std::string imu_bag_path;
    std::string imu_topic;
    std::string rerun_blue_print_path;
    bool solve_time_offset = false;
    bool is_valid = false;
};
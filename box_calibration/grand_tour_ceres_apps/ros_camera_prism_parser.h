#pragma once

#include <string>
#include <vector>

struct ROSCameraPrismParser {
    ROSCameraPrismParser(std::string program_name, int argc, char *argv[]);
    std::string cameras_calibration_path;
    std::string output_path;
    std::vector<std::string> camera_bag_paths;
    std::string prism_bag_path;
    std::string prism_topic;
    bool solve_time_offset = false;
    bool is_valid = false;
};
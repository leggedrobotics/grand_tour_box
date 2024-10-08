//
// Created by fu on 27/09/24.
//

#include "ros_parsers.h"
#include <ros/package.h>
#include <filesystem>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

OnlineCameraCameraParser::OnlineCameraCameraParser(int argc, char **argv) {
    const std::string package_name = "grand_tour_ceres_apps";
    std::string package_path = ros::package::getPath(package_name);
    std::string initial_guess_default_path = package_path + "/config/initial_guess.yaml";
    std::string frameid_mapping_default_path = package_path + "/config/rostopic_frameid_mappings.yaml";

    argparse::ArgumentParser program("OnlineCameraCameraCalibration");
    program.add_argument("-i", "--initial_guess")
            .default_value(initial_guess_default_path)
            .help("File containing kalibr-style initial guess of intrinsics and extrinsics."
                  "With each camN key specifying a ROSTOPIC");
    program.add_argument("-m", "--rostopic_frameid_mapping")
            .default_value(frameid_mapping_default_path)
            .help("File containing keys mapping rostopic names to their corresponding frame ids");
    try {
        program.parse_args(argc, argv);
    }
    catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return;
    }
    initial_guess_path = program.get<std::string>("i");
    rostopic_frameid_mapping_path = program.get<std::string>("m");
    is_valid = fs::exists(initial_guess_path);
    if (!is_valid) {
        ROS_ERROR_STREAM("The path: " + initial_guess_path + " does not exist");
    }
    is_valid = is_valid && fs::exists(rostopic_frameid_mapping_path);
    if (!is_valid) {
        ROS_ERROR_STREAM("The path: " + rostopic_frameid_mapping_path + " does not exist");
    }
}

std::map<std::string, std::string> LoadRostopicFrameIDMapping(const std::string yaml_path) {
    const YAML::Node yaml_mapping = YAML::LoadFile(yaml_path);
    std::map<std::string, std::string> output;
    for (YAML::const_iterator it = yaml_mapping.begin(); it != yaml_mapping.end(); ++it) {
        const auto &rostopic = it->first.as<std::string>();
        const auto &frameid = it->second.as<std::string>();
        output[rostopic] = frameid;
    }
    return output;
}

//
// Created by fu on 27/09/24.
//

#ifndef GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_PARSER_H
#define GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_PARSER_H


#include <gtboxcalibration/argparse.h>

struct ROSCameraCameraParser {
    ROSCameraCameraParser(std::string program_name, int argc, char **argv);
    ROSCameraCameraParser(std::string program_name, int argc, char **argv, bool is_online);
    std::string initial_guess_path;
    std::string rostopic_frameid_mapping_path;
    std::vector<std::string> bag_paths;
    bool is_valid = false;
    std::string output_path;
    std::string rerun_blue_print_path;
};

std::map<std::string, std::string> LoadRostopicFrameIDMapping(const std::string yaml_path);

#endif //GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_PARSER_H

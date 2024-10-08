//
// Created by fu on 27/09/24.
//

#ifndef GRAND_TOUR_CERES_APPS_ROS_PARSERS_H
#define GRAND_TOUR_CERES_APPS_ROS_PARSERS_H


#include <gtboxcalibration/argparse.h>

struct OnlineCameraCameraParser {
    OnlineCameraCameraParser(int argc, char **argv);
    std::string initial_guess_path;
    std::string rostopic_frameid_mapping_path;
    bool is_valid = false;
};

std::map<std::string, std::string> LoadRostopicFrameIDMapping(const std::string yaml_path);

#endif //GRAND_TOUR_CERES_APPS_ROS_PARSERS_H

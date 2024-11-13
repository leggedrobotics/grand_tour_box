//
// Created by fu on 27/09/24.
//

#include "ros_camera_camera_online_program.h"
#include "ros_camera_camera_parser.h"
#include <ros/ros.h>

int main(int argc, char **argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "camera_camera_online_calibration");
    ROSCameraCameraParser parser("camera_camera_online_calibration", argc, argv);
    if (!parser.is_valid) {
        return -1;
    }
    ROSCameraCameraOnlineProgram program(parser);
    if (!program.isValid()) {
        return -1;
    }

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    program.run();

    return 0;
}

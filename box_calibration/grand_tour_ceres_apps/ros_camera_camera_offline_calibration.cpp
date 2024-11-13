//
// Created by fu on 27/09/24.
//

#include "ros_camera_camera_offline_program.h"
#include "ros_camera_camera_parser.h"
#include <ros/ros.h>

int main(int argc, char **argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "camera_camera_offline_calibration");
    ROSCameraCameraParser parser("camera_camera_offline_calibration", argc, argv);
    if (!parser.is_valid) {
        return -1;
    }
    ROSCameraCameraOfflineProgram program(parser);
    ROS_INFO_STREAM("Parsed");
    ROS_INFO_STREAM("Running");
    program.run();

    return 0;
}

/**
 * @file main.cpp
 * @brief Entry point for the offline camera-to-camera calibration node.
 *
 * Initializes the ROS node, parses command-line arguments, and runs the offline
 * calibration program.
 *
 * Created on: 27/09/2024
 * Author: Lanke Frank Tarimo Fu
 * License: MIT
 */

#include "ros_camera_camera_offline_program.h"
#include "ros_camera_camera_parser.h"

#include <ros/ros.h>
#include <cstdlib>   // for EXIT_SUCCESS / EXIT_FAILURE
#include <exception> // for std::exception

int main(int argc, char** argv) {
    constexpr char kNodeName[] = "camera_camera_offline_calibration";

    // Initialize the ROS node
    ros::init(argc, argv, kNodeName);

    try {
        // Parse CLI / ROS params
        ROSCameraCameraParser parser(kNodeName, argc, argv);
        if (!parser.is_valid) {
            ROS_ERROR_STREAM("Argument/parameter parsing failed. Exiting.");
            return EXIT_FAILURE;
        }

        // Run the offline calibration program
        ROS_INFO_STREAM("Parsed arguments successfully.");
        ROS_INFO_STREAM("Starting offline camera calibration...");

        ROSCameraCameraOfflineProgram program(parser);

        if (!program.run()) {
            return EXIT_FAILURE;
        }

        ROS_INFO_STREAM("Offline camera calibration finished.");
        return EXIT_SUCCESS;

    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Unhandled exception: " << e.what());
        return EXIT_FAILURE;
    } catch (...) {
        ROS_ERROR_STREAM("Unhandled unknown exception.");
        return EXIT_FAILURE;
    }
}

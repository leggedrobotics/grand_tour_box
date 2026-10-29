//
// Created by fu on 27/09/24.
//

#include "ros_camera_camera_online_program.h"
#include "ros_camera_camera_parser.h"
#include "rerun_camera_camera_viz.h"
#include <ros/ros.h>
#include "rerun_recording.h"

int main(int argc, char **argv) {

    constexpr char kNodeName[] = "camera_camera_online_calibration";


    // Initialize the ROS node
    ros::init(argc, argv, kNodeName);
    ROSCameraCameraParser parser(kNodeName, argc, argv, true);
    if (!parser.is_valid) {
        return -1;
    }
    ROSCameraCameraOnlineProgram program(parser);
    auto rec = openRerunRecording(kNodeName, parser.visualize, parser.rerun_recording,
                                  parser.rerun_blue_print_path);
    if (rec) program.setViz(std::make_unique<RerunCameraCameraViz>(*rec));
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

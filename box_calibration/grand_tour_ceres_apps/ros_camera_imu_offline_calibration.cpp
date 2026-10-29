#include "rerun_camera_imu_viz.h"
#include "rerun_recording.h"
#include "ros_camera_imu_parser.h"
#include "ros_camera_imu_program.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char **argv) {
    constexpr char kNodeName[] = "camera_imu_offline_calibration";
    ros::init(argc, argv, kNodeName);
    try {
        ROSCameraIMUParser parser(kNodeName, argc, argv);
        if (!parser.is_valid)
            return 1;
        auto rec = openRerunRecording(kNodeName, parser.visualize, parser.rerun_recording,
                                      parser.rerun_blue_print_path);
        std::unique_ptr<CameraImuVizInterface> viz;
        if (rec) {
            rec->log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);
            viz = std::make_unique<RerunCameraImuViz>(*rec);
        }
        ROSCameraIMUProgram program(parser, std::move(viz));
        return program.isValid() && program.Solve() ? 0 : 1;
    } catch (const std::exception &error) {
        std::cerr << "Camera IMU calibration failed: " << error.what() << '\n';
        return 1;
    }
}

#include "rerun_camera_prism_viz.h"
#include "rerun_recording.h"
#include "ros_camera_prism_parser.h"
#include "ros_camera_prism_program.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
    constexpr char name[] = "camera_prism_offline_calibration";
    ros::init(argc, argv, name);
    try {
        ROSCameraPrismParser parser(name, argc, argv);
        if (!parser.is_valid) return 1;
        auto recording = openRerunRecording(name, parser.visualize, parser.rerun_recording, parser.rerun_blue_print_path);
        std::unique_ptr<CameraPrismVizInterface> viz;
        if (recording) viz = std::make_unique<RerunCameraPrismViz>(*recording);
        ROSCameraPrismProgram program(parser, std::move(viz));
        return program.Solve() && program.WriteOutputParameters() ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Prism calibration failed: " << error.what() << '\n';
        return 1;
    }
}

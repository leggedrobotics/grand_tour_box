
#include "ros_camera_prism_parser.h"
#include "ros_camera_prism_program.h"

#include <rerun.hpp>
#include "rerun_camera_prism_viz.h"

int main(int argc, char **argv) {
    constexpr char kNodeName[] = "camera_prism_offline_calibration";
    auto rec = rerun::RecordingStream(kNodeName);
    rec.spawn().exit_on_failure();
    ROSCameraPrismParser parser(kNodeName, argc, argv);
    if (!parser.is_valid) return 1;

    rec.log_file_from_path(parser.rerun_blue_print_path);
    ROSCameraPrismProgram program(parser, std::make_unique<RerunCameraPrismViz>(rec));
    program.Solve();
    program.WriteOutputParameters();
    return 0;
}

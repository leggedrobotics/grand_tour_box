
#include "ros_camera_imu_parser.h"
#include "ros_camera_imu_program.h"

#include <rerun.hpp>
#include "rerun_camera_imu_viz.h"
#include <memory>

int main(int argc, char **argv) {
    constexpr char kNodeName[] = "camera_imu_offline_calibration";
    auto rec = rerun::RecordingStream(kNodeName);
    rec.spawn().exit_on_failure();
    rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN); // Set an up-axis
    ROSCameraIMUParser parser(kNodeName, argc, argv);
    if (!parser.is_valid) return 1;

    rec.log_file_from_path(parser.rerun_blue_print_path);
    ROSCameraIMUProgram program(parser, std::make_unique<RerunCameraImuViz>(rec));
    if (!program.isValid()) {
        std::cerr << "Camera IMU calibration failed for " << parser.imu_topic << std::endl;
        return -1;
    }
    program.Solve();
    return 0;
}


#include "ros_camera_prism_parser.h"
#include "ros_camera_prism_program.h"
int main(int argc, char **argv) {
    ROSCameraPrismParser parser("camera_prism_offline", argc, argv);
    if (!parser.is_valid) return 1;

    ROSCameraPrismProgram program(parser);
    program.Solve();
    program.ResetAndRepopulateProblem();
    program.Solve();
    program.WriteOutputParameters();
    return 0;
}

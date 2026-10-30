//
// Created by fu on 05/09/2024.
//

#include <gtboxcalibration/argparsers.h>
#include <gtboxcalibration/ceresprograms.h>


int main(int argc, char **argv) {
    try {
        CameraPrismCalibrationAppParser parser(argc, argv);
        CameraPrismProgram program(parser);
        return program.Solve() && program.WriteOutputParameters() ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Prism calibration failed: " << error.what() << '\n';
        return 1;
    }
}

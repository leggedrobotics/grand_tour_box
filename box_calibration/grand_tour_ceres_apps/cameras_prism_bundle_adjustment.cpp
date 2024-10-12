//
// Created by fu on 05/09/2024.
//

#include <gtboxcalibration/argparsers.h>
#include <gtboxcalibration/ceresprograms.h>


int main(int argc, char **argv) {
    CameraPrismCalibrationAppParser cameraPrismCalibrationAppParser(argc, argv);
    CameraPrismProgram program(cameraPrismCalibrationAppParser);

    program.Solve();
    program.ResetAndRepopulateProblem();
    program.Solve();
    program.WriteOutputParameters();
    return 0;
}

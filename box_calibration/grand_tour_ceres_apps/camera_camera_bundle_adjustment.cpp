//
// Created by fu on 02/08/2024.
//


#include <fstream>

#include <gtboxcalibration/argparsers.h>
#include <gtboxcalibration/ceresprograms.h>
#include <gtboxcalibration/json.h>
#include <gtboxcalibration/utils.h>

#include <ceres/ceres.h>
#include <gtboxcalibration/ceresproblems.h>


using json = nlohmann::json;

int main(int argc, char *argv[]) {
    const auto args = CameraCameraCalibrationAppParser(argc, argv);
    if (!args.is_valid) {
        return -1;
    }

    auto program = CameraCameraProgram(args);

    program.SetPresolveCameraExtrinsicsConstants();
    program.Solve();
    program.ResetAndRepopulateProblem();
    program.Solve();

    return 0;
}

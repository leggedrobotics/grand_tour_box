//
// Created by fu on 05/09/2024.
//

#include <gtboxcalibration/argparsers.h>
#include <gtboxcalibration/visualization.h>
#include <gtboxcalibration/ceresprograms.h>


int main(int argc, char **argv) {
    CameraPrismCalibrationAppParser cameraPrismCalibrationAppParser(argc, argv);
    CameraPrismProgram program(cameraPrismCalibrationAppParser);
    CameraPrismVisualization viz("", "camera-prism-opt", program);

    viz.PlotState("initial_guess");
    program.Solve();
    program.ResetAndRepopulateProblem();
    program.Solve();
    program.WriteOutputParameters();
    viz.PlotState("optimized");
    viz.PlotCovariance();
//    std::map<std::string, std::map<unsigned long, std::pair<Eigen::Vector3d, double>>> residuals_and_losses;
//    for (const auto&[camera_name, camera_residual_block_at_time]: residual_block_map) {
//        for (const auto&[camera_stamp, block_id]: camera_residual_block_at_time) {
//            residuals_and_losses[camera_name][camera_stamp] = GetResidualsAndLossAtResidualBlockID(problem.getProblem(), block_id);
//            residuals_and_losses[camera_name][camera_stamp].second = std::sqrt(
//                    residuals_and_losses[camera_name][camera_stamp].first.dot(
//                            residuals_and_losses[camera_name][camera_stamp].first)
//            );
//        }
//    }
//    viz.PlotResiduals(residuals_and_losses);
//    std::cout << prism_board_setup.t_offset[0] << std::endl;
//    std::cout << Eigen::Map<Eigen::Vector3d>(prism_board_setup.t_cam0_prism) << std::endl;
    return 0;
}
//
// Created by fu on 09/09/2024.
//

#include <gtboxcalibration/prismpositionresiduals.h>

PrismInCam0InBoardInTotalStationConsistencyError::PrismInCam0InBoardInTotalStationConsistencyError(
        const Eigen::Affine3d &tCamselfCam0, const Eigen::Affine3d &tBoardCamself,
        unsigned long long camera_detection_time,
        const PrismPositionDetectionData &prism_detections) : T_camself_cam0_(tCamselfCam0),
                                                              T_board_camself_(tBoardCamself),
                                                              camera_detection_time_(
                                                                      camera_detection_time),
                                                              total_station_prism_measurement_data_(
                                                                      prism_detections) {

}

std::pair<Eigen::VectorXd, double>
GetResidualsAndLossAtResidualBlockID(const ceres::Problem &problem, ceres::ResidualBlockId residualBlockId) {

    const ceres::CostFunction *cost_function = problem.GetCostFunctionForResidualBlock(residualBlockId);
    const ceres::LossFunction *loss_function = problem.GetLossFunctionForResidualBlock(residualBlockId);

    // Retrieve parameter blocks
    std::vector<double *> parameter_blocks;
    problem.GetParameterBlocksForResidualBlock(residualBlockId, &parameter_blocks);

    // Allocate space for residuals
    std::vector<double> block_residuals(cost_function->num_residuals());

    // Evaluate the residuals
    cost_function->Evaluate(parameter_blocks.data(), block_residuals.data(), nullptr);

    // Compute the squared norm of the residuals
    double squared_norm = 0;
    for (const double &residual: block_residuals) {
        squared_norm += residual * residual;
    }

    // Now apply the loss function to get the effective loss value
    double rho[3];  // rho[0] is the actual loss, rho[1] is the derivative, rho[2] is the second derivative
    if (loss_function != nullptr) {
        loss_function->Evaluate(squared_norm, rho);
    } else {
        rho[0] = squared_norm;  // If no loss function, the loss is just the squared residual
    }

    Eigen::Map<Eigen::Vector3d> residuals(block_residuals.data());
    return std::make_pair(residuals, rho[0]);
}

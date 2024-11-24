//
// Created by fu on 13/09/2024.
//

#include "gtboxcalibration/ceresproblems.h"

CeresProblem::CeresProblem() {
    problem_options_.enable_fast_removal = true;
    problem_ = ceres::Problem(problem_options_);
    solver_options_.num_threads = 12;
    solver_options_.update_state_every_iteration = false;
    solver_options_.linear_solver_type = ceres::SPARSE_SCHUR;
    solver_options_.minimizer_progress_to_stdout = false;
    solver_options_.max_num_iterations = 20;
}

void CeresProblem::PlotCovariance() {

    // Step 3: Prepare to compute the covariance matrix
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);

    // Get all parameter blocks
    std::vector<double *> parameter_blocks;
    problem_.GetParameterBlocks(&parameter_blocks);

    // Specify pairs of parameter blocks (auto-iterate over all parameter blocks)
    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    for (size_t i = 0; i < parameter_blocks.size(); ++i) {
        for (size_t j = i; j < parameter_blocks.size(); ++j) {
            covariance_blocks.push_back(std::make_pair(parameter_blocks[i], parameter_blocks[j]));
        }
    }

    // Compute the covariance matrix
    if (!covariance.Compute(covariance_blocks, &problem_)) {
        std::cout << "Failed to compute covariance matrix.\n";
    }

    // Step 4: Output the dense covariance matrix
    std::cout << "Dense covariance matrix:\n";

    std::vector<unsigned int> parameter_block_start_index(1, 0);
    for (size_t i = 0; i < parameter_blocks.size(); ++i) {
        unsigned int prev = parameter_block_start_index.back();
        parameter_block_start_index.push_back(problem_.ParameterBlockSize(parameter_blocks[i]) + prev);
    }
    // Loop through all parameter block pairs and print the covariance matrix
    Eigen::MatrixXd full_matrix(parameter_block_start_index.back(), parameter_block_start_index.back());
    full_matrix.setZero();

    for (size_t i = 0; i < parameter_blocks.size(); ++i) {
        for (size_t j = 0; j < parameter_blocks.size(); ++j) {
            int block_size_i = problem_.ParameterBlockSize(parameter_blocks[i]);
            int block_size_j = problem_.ParameterBlockSize(parameter_blocks[j]);
            Eigen::MatrixXd covariance_matrix(block_size_i, block_size_j);
            if (covariance.GetCovarianceBlock(parameter_blocks[i], parameter_blocks[j], covariance_matrix.data())) {
                covariance_matrix = covariance_matrix.transpose();
                full_matrix.block(parameter_block_start_index[i],
                                  parameter_block_start_index[j],
                                  block_size_i,
                                  block_size_j) = covariance_matrix;
            }
        }
    }
    std::cout << full_matrix << std::endl;
}

bool CeresProblem::ComputeAndFetchCovariance(const std::vector<const double *> &parameter_blocks,
                                             Eigen::MatrixXd &full_matrix) {
    const auto covariance = this->ComputeCovariance(parameter_blocks);
    if (covariance == nullptr) return false;
    return this->FetchCovariance(covariance, parameter_blocks, full_matrix);
}

std::shared_ptr<ceres::Covariance>
CeresProblem::ComputeCovariance(const std::vector<const double *> &parameter_blocks) {
    ceres::Covariance::Options covariance_options;
    covariance_options.algorithm_type = ceres::SPARSE_QR;
    covariance_options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
    covariance_options.apply_loss_function = false;
    covariance_options.num_threads = 4;
    covariance_options.apply_loss_function = true;
    auto covariance = std::make_shared<ceres::Covariance>(covariance_options);
    // Compute the covariance matrix
    if (!covariance->Compute(parameter_blocks, &problem_)) {
        std::cout << "Failed to compute covariance matrix.\n";
        return nullptr;
    }
    return covariance;
}

bool CeresProblem::FetchCovariance(const std::shared_ptr<ceres::Covariance> covariance,
                                   const std::vector<const double *> &parameter_blocks, Eigen::MatrixXd &full_matrix) {
    unsigned int covariance_size = 0;
    for (size_t i = 0; i < parameter_blocks.size(); ++i) {
        covariance_size += problem_.ParameterBlockTangentSize(parameter_blocks[i]);
    }
    // Loop through all parameter block pairs and print the covariance matrix
    full_matrix = Eigen::MatrixXd(covariance_size, covariance_size);
    full_matrix.setZero();
    covariance->GetCovarianceMatrixInTangentSpace(parameter_blocks,
                                                  full_matrix.data());
    return true;
}

bool CeresProblem::FetchSubBlockCovariance(const std::shared_ptr<ceres::Covariance> covariance,
                                           const std::vector<const double *> parameter_blocks,
                                           std::vector<Eigen::MatrixXd> &block_matrices) {
    block_matrices.clear();
    for (const auto& param_block : parameter_blocks) {
        unsigned int covariance_size = problem_.ParameterBlockTangentSize(param_block);
        Eigen::MatrixXd block_covariance = Eigen::MatrixXd(covariance_size, covariance_size);
        block_covariance.setZero();
        covariance->GetCovarianceBlockInTangentSpace(param_block, param_block,
                                                     block_covariance.data());
        block_matrices.push_back(block_covariance);
    }
    return true;
}

std::shared_ptr<ceres::Covariance>
CeresProblem::ComputeSubBlockCovariance(const std::vector<const double *> &param_blocks) {
    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    for (const auto& param : param_blocks) {
        covariance_blocks.push_back(std::make_pair(param, param));
    }
    ceres::Covariance::Options covariance_options;
    covariance_options.algorithm_type = ceres::SPARSE_QR;
    covariance_options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
    covariance_options.apply_loss_function = false;
    covariance_options.num_threads = 4;
    auto covariance = std::make_shared<ceres::Covariance>(covariance_options);
    // Compute the covariance matrix
    if (!covariance->Compute(covariance_blocks, &problem_)) {
        std::cout << "Failed to compute covariance matrix.\n";
        return nullptr;
    }
    return covariance;
}


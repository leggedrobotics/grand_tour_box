//
// Created by fu on 13/09/2024.
//

#ifndef GRAND_TOUR_CERES_APPS_CERESPROBLEMS_H
#define GRAND_TOUR_CERES_APPS_CERESPROBLEMS_H

#include <ceres/ceres.h>

struct CeresProblem {

    CeresProblem();

    ceres::Problem &getProblem() { return problem_; }

    void PlotCovariance();

    bool ComputeAndFetchCovariance(const std::vector<const double *> &parameter_blocks, Eigen::MatrixXd &full_matrix);

    std::shared_ptr<ceres::Covariance> ComputeCovariance(const std::vector<const double *> &parameter_blocks);

    std::shared_ptr<ceres::Covariance>
    ComputeSubBlockCovariance(const std::vector<const double *> &param_blocks);


    bool FetchCovariance(const std::shared_ptr<ceres::Covariance>,
                         const std::vector<const double *> &parameter_blocks, Eigen::MatrixXd &full_matrix);

    bool FetchSubBlockCovariance(const std::shared_ptr<ceres::Covariance>,
                                 const std::vector<const double *> parameter_blocks,
                                 std::vector<Eigen::MatrixXd> &block_matrices);

    ceres::Problem problem_;
    ceres::Problem::Options problem_options_;
    ceres::Solver::Options solver_options_;
    ceres::Solver::Summary summary_;
};


#endif //GRAND_TOUR_CERES_APPS_CERESPROBLEMS_H

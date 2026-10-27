#pragma once

#include <Eigen/Core>
#include <limits>

struct DenseQpResult {
    bool solved = false;
    Eigen::VectorXd step;
    double primal_residual = std::numeric_limits<double>::infinity();
    double stationarity_residual = std::numeric_limits<double>::infinity();
    double complementarity_residual = std::numeric_limits<double>::infinity();
    int iterations = 0;
    bool timed_out = false;
};

// min 0.5*x'H*x+g'x, subject to C*x <= bounds; H must be positive definite.
// Primal tolerance is absolute; dual/complementarity tests scale with the
// problem's gradient/objective magnitudes. Reported residuals remain unscaled.
DenseQpResult solve_dense_qp(const Eigen::MatrixXd& H, const Eigen::VectorXd& g,
    const Eigen::MatrixXd& C, const Eigen::VectorXd& bounds,
    int max_iterations = 2000, double tolerance = 1e-7, double max_seconds = 0.0);

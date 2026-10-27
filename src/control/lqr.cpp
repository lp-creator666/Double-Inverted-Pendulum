#include "control/lqr.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace {

using Matrix6 = Eigen::Matrix<double, kStateDimension, kStateDimension>;
using Vector6 = Eigen::Matrix<double, kStateDimension, 1>;
using Row6 = Eigen::Matrix<double, 1, kStateDimension>;

using RowMajorMatrix6 = Eigen::Matrix<
    double, kStateDimension, kStateDimension, Eigen::RowMajor
>;

double max_abs(const Matrix6& matrix) {
    return matrix.cwiseAbs().maxCoeff();
}
}
LqrResult solve_discrete_lqr(
    const MatrixA& A,
    const VectorB& B,
    const LqrConfig& config
) {
    if (!std::isfinite(config.r) || config.r <= 0.0) {
        throw std::invalid_argument("LQR R must be positive and finite");
    }

    if (config.max_iterations <= 0 ||
        !std::isfinite(config.convergence_tolerance) ||
        config.convergence_tolerance <= 0.0 ||
        config.convergence_tolerance >= 1.0) {
        throw std::invalid_argument("Invalid LQR iteration settings");
    }

    for (double weight : config.q_diagonal) {
        if (!std::isfinite(weight) || weight <= 0.0) {
            throw std::invalid_argument(
                "LQR diagonal Q weights must be positive and finite"
            );
        }
    }

    const auto scaled = scale_linearization(A, B, config.scaling);

    const Matrix6 As =
        Eigen::Map<const RowMajorMatrix6>(scaled.A.data());

    const Vector6 Bs =
        Eigen::Map<const Vector6>(scaled.B.data());

    Matrix6 Q = Matrix6::Zero();

    for (int i = 0; i < kStateDimension; ++i) {
        Q(i, i) = config.q_diagonal[i];
    }

    Matrix6 P = Q;
    LqrResult result;
    bool converged = false;

    for (int iteration = 1;
         iteration <= config.max_iterations;
         ++iteration) {

        const double denominator =
            config.r + (Bs.transpose() * P * Bs)(0, 0);

        if (!std::isfinite(denominator) || denominator <= 0.0) {
            throw std::runtime_error("Invalid Riccati denominator");
        }

        const Row6 K =
            (Bs.transpose() * P * As) / denominator;

        Matrix6 next =
            Q + As.transpose() * P * As
              - As.transpose() * P * Bs * K;

        // 理论上 P 对称；消除浮点计算带来的微小不对称。
        next = (0.5 * (next + next.transpose())).eval();

        if (!next.allFinite()) {
            throw std::runtime_error("Riccati iteration became non-finite");
        }

        result.final_change =
            max_abs(next - P) / std::max(1.0, max_abs(next));

        P = next;
        result.iterations = iteration;

        if (result.final_change <= config.convergence_tolerance) {
            converged = true;
            break;
        }
    }

    if (!converged) {
        throw std::runtime_error("Riccati iteration did not converge");
    }

    const double denominator =
        config.r + (Bs.transpose() * P * Bs)(0, 0);

    if (!std::isfinite(denominator) || denominator <= 0.0) {
        throw std::runtime_error("Invalid final Riccati denominator");
    }

    const Row6 K =
        (Bs.transpose() * P * As) / denominator;

    const Matrix6 residual =
        Q + As.transpose() * P * As
          - As.transpose() * P * Bs * K - P;

    if (!K.allFinite() || !residual.allFinite()) {
        throw std::runtime_error("Non-finite LQR result");
    }

    result.riccati_residual =
        max_abs(residual) / std::max(1.0, max_abs(P));

    result.symmetry_error =
        max_abs(P - P.transpose()) / std::max(1.0, max_abs(P));

    Eigen::SelfAdjointEigenSolver<Matrix6> p_solver(P);
    if (p_solver.info() != Eigen::Success) {
        throw std::runtime_error("P eigenvalue calculation failed");
    }
    result.min_p_eigenvalue = p_solver.eigenvalues().minCoeff();

    const Matrix6 closed_loop = As - Bs * K;
    Eigen::EigenSolver<Matrix6> closed_loop_solver(closed_loop, false);
    if (closed_loop_solver.info() != Eigen::Success) {
        throw std::runtime_error("Closed-loop eigenvalue calculation failed");
    }
    result.closed_loop_radius =
        closed_loop_solver.eigenvalues().cwiseAbs().maxCoeff();

    if (!std::isfinite(result.riccati_residual) ||
        !std::isfinite(result.symmetry_error) ||
        !std::isfinite(result.min_p_eigenvalue) ||
        !std::isfinite(result.closed_loop_radius) ||
        result.riccati_residual > 1e-9 ||
        result.symmetry_error > 1e-12 ||
        result.min_p_eigenvalue <= 0.0 ||
        result.closed_loop_radius >= 1.0) {
        throw std::runtime_error("LQR numerical verification failed");
    }

    for (int row = 0; row < kStateDimension; ++row) {
        result.K[row] = K(0, row);

        for (int column = 0; column < kStateDimension; ++column) {
            result.P[row * kStateDimension + column] = P(row, column);
        }
    }

    return result;
}


double compute_lqr_force(
    const State& state,
    const LqrResult& lqr,
    const LqrConfig& config
) {
    if (!state.finite()) {
        throw std::invalid_argument("Non-finite LQR state");
    }

    const StateVector x{
        state.p, state.dp,
        state.q1, state.dq1,
        state.q2, state.dq2
    };

    double v = 0.0;

    for (int i = 0; i < kStateDimension; ++i) {
        const double scale = config.scaling.state_scale[i];

        if (!std::isfinite(scale) || scale <= 0.0 ||
            !std::isfinite(lqr.K[i])) {
            throw std::invalid_argument("Invalid LQR gain or scale");
        }

        const double z = x[i] / scale;
        v -= lqr.K[i] * z;
    }

    const double input_scale = config.scaling.input_scale;

    if (!std::isfinite(input_scale) || input_scale <= 0.0) {
        throw std::invalid_argument("Invalid LQR input scale");
    }

    const double force = input_scale * v;

    if (!std::isfinite(force)) {
        throw std::runtime_error("Non-finite LQR force");
    }

    return force;
}

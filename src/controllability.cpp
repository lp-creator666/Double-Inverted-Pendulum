#include "controllability.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

ControllabilityResult analyze_controllability(
    const MatrixA& A, const VectorB& B, double tolerance
) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || tolerance >= 1.0) {
        throw std::invalid_argument("SVD relative tolerance must be in (0, 1)");
    }

    using Matrix6 = Eigen::Matrix<double, kStateDimension, kStateDimension>;
    using Vector6 = Eigen::Matrix<double, kStateDimension, 1>;
    const Eigen::Map<const Eigen::Matrix<double, kStateDimension,
        kStateDimension, Eigen::RowMajor>> state_matrix(A.data());
    const Eigen::Map<const Vector6> input_vector(B.data());
    if (!state_matrix.allFinite() || !input_vector.allFinite()) {
        throw std::invalid_argument("A or B contains non-finite values");
    }

    Matrix6 controllability;
    Vector6 column = input_vector;
    for (int index = 0; index < kStateDimension; ++index) {
        controllability.col(index) = column;
        if (index + 1 < kStateDimension) {
            column = (state_matrix * column).eval();
        }
    }
    if (!controllability.allFinite()) {
        throw std::overflow_error("Controllability matrix overflow");
    }

    // Direct SVD avoids squaring the condition number by forming C^T C.
    Eigen::JacobiSVD<Matrix6> svd(controllability);
    svd.setThreshold(tolerance);
    if (svd.info() != Eigen::Success) {
        throw std::runtime_error("Controllability SVD did not converge");
    }

    ControllabilityResult result;
    for (int row = 0; row < kStateDimension; ++row) {
        result.singular_values[row] = svd.singularValues()[row];
        for (int col = 0; col < kStateDimension; ++col) {
            result.matrix[row * kStateDimension + col] = controllability(row, col);
        }
    }
    result.rank = static_cast<int>(svd.rank());
    const double smallest = result.singular_values.back();
    result.condition_number = smallest > 0.0
        ? result.singular_values.front() / smallest
        : std::numeric_limits<double>::infinity();
    return result;
}

namespace {
using RealMatrix = Eigen::Matrix<double, kStateDimension, kStateDimension>;
using RealVector = Eigen::Matrix<double, kStateDimension, 1>;

void validate_model(const MatrixA& A, const VectorB& B, double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || tolerance >= 1.0)
        throw std::invalid_argument("SVD tolerance must be in (0, 1)");
    for (double value : A)
        if (!std::isfinite(value)) throw std::invalid_argument("Non-finite A");
    for (double value : B)
        if (!std::isfinite(value)) throw std::invalid_argument("Non-finite B");
}
RealMatrix as_matrix(const MatrixA& A) {
    return Eigen::Map<const Eigen::Matrix<double, kStateDimension,
        kStateDimension, Eigen::RowMajor>>(A.data());
}
} // namespace

HorizonControllability analyze_controllability_horizon(
    const MatrixA& A, const VectorB& B, int steps, double tolerance
) {
    validate_model(A, B, tolerance);
    if (steps < kStateDimension)
        throw std::invalid_argument("Horizon needs at least six steps");
    const RealMatrix state_matrix = as_matrix(A);
    RealVector column = Eigen::Map<const RealVector>(B.data());
    Eigen::MatrixXd reachability(kStateDimension, steps);
    for (int index = 0; index < steps; ++index) {
        reachability.col(index) = column;
        if (index + 1 < steps) column = (state_matrix * column).eval();
    }
    if (!reachability.allFinite())
        throw std::overflow_error("Horizon matrix overflow");
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(reachability);
    if (svd.info() != Eigen::Success)
        throw std::runtime_error("Horizon SVD did not converge");
    svd.setThreshold(tolerance);
    HorizonControllability result;
    result.steps = steps;
    result.rank = static_cast<int>(svd.rank());
    for (int index = 0; index < kStateDimension; ++index)
        result.singular_values[index] = svd.singularValues()[index];
    const double smallest = result.singular_values.back();
    result.condition_number = smallest > 0.0
        ? result.singular_values.front() / smallest
        : std::numeric_limits<double>::infinity();
    return result;
}

StabilizabilityResult analyze_stabilizability(
    const MatrixA& A, const VectorB& B, double tolerance, double boundary_margin
) {
    validate_model(A, B, tolerance);
    if (!std::isfinite(boundary_margin) || boundary_margin < 0.0 ||
        boundary_margin >= 1.0)
        throw std::invalid_argument("Invalid stability boundary margin");
    const RealMatrix state_matrix = as_matrix(A);
    Eigen::ComplexEigenSolver<RealMatrix> eigen_solver(state_matrix, false);
    if (eigen_solver.info() != Eigen::Success)
        throw std::runtime_error("Eigenvalue computation did not converge");
    const Eigen::MatrixXcd complex_state = state_matrix.cast<std::complex<double>>();
    const Eigen::VectorXcd complex_input =
        Eigen::Map<const RealVector>(B.data()).cast<std::complex<double>>();
    StabilizabilityResult result;
    result.stabilizable = true;
    for (int index = 0; index < kStateDimension; ++index) {
        auto& mode = result.modes[index];
        mode.eigenvalue = eigen_solver.eigenvalues()[index];
        mode.requires_control = std::abs(mode.eigenvalue) >= 1.0 - boundary_margin;
        Eigen::MatrixXcd pbh(kStateDimension, kStateDimension + 1);
        pbh.leftCols(kStateDimension) = mode.eigenvalue *
            Eigen::MatrixXcd::Identity(kStateDimension, kStateDimension) - complex_state;
        pbh.col(kStateDimension) = complex_input;
        Eigen::JacobiSVD<Eigen::MatrixXcd> svd(pbh);
        if (svd.info() != Eigen::Success)
            throw std::runtime_error("PBH SVD did not converge");
        svd.setThreshold(tolerance);
        mode.rank = static_cast<int>(svd.rank());
        mode.singular_value_ratio = svd.singularValues()[0] > 0.0
            ? svd.singularValues()[kStateDimension - 1] / svd.singularValues()[0]
            : 0.0;
        if (mode.requires_control && mode.rank < kStateDimension)
            result.stabilizable = false;
    }
    return result;
}

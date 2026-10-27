#pragma once

#include <array>
#include <complex>
#include "model/linearize.hpp"

struct ControllabilityResult {
    MatrixA matrix{};
    std::array<double, kStateDimension> singular_values{};
    int rank = 0;
    double condition_number = 0.0;
};

// Rank counts singular values greater than tolerance * largest singular value.
ControllabilityResult analyze_controllability(
    const MatrixA& A,
    const VectorB& B,
    double tolerance = 1e-10
);

struct HorizonControllability {
    std::array<double, kStateDimension> singular_values{};
    int steps = 0;
    int rank = 0;
    double condition_number = 0.0;
};

// Uses the original A/B with an independently updated input at every step.
HorizonControllability analyze_controllability_horizon(
    const MatrixA& A, const VectorB& B, int steps,
    double tolerance = 1e-10
);

struct PBHMode {
    std::complex<double> eigenvalue{};
    double singular_value_ratio = 0.0;
    int rank = 0;
    bool requires_control = false;
};

struct StabilizabilityResult {
    std::array<PBHMode, kStateDimension> modes{};
    bool stabilizable = false;
};

// Boundary eigenvalues are included conservatively: |lambda| >= 1 - margin.
StabilizabilityResult analyze_stabilizability(
    const MatrixA& A, const VectorB& B,
    double tolerance = 1e-10, double boundary_margin = 1e-9
);

// Checks the upright, zero-input model; uses only the independent prediction data.
void run_model_diagnostics(const mjModel* model, mjData* prediction_data,
    const int* qpos_addresses, const int* qvel_addresses,
    const Linearization& linearization);

#pragma once

#include <array>
#include <vector>
#include "types.hpp"
#include "optimization/nonlinear_model.hpp"

struct NmpcCostConfig {
    // Physical coordinates: p, dp, q1, dq1, q1+q2, dq1+dq2.
    // Initial weights; not the scaled LQR Q and not yet tuned for swing-up.
    std::array<double, 6> state_weights{10.0, 1.0, 100.0, 1.0, 100.0, 1.0};
    double input_weight = 0.01;
    double input_change_weight = 0.1;
    double terminal_weight = 5.0;
};

// Upright, stationary, cart-centered target. Angles are in radians.
double compute_nmpc_state_cost(const State& state, const NmpcCostConfig& config);

double compute_nmpc_trajectory_cost(
    const NmpcPrediction& prediction,
    const std::vector<double>& forces,
    const NmpcCostConfig& config,
    double previous_force = 0.0
);

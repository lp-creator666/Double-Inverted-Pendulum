#pragma once

#include <vector>
#include <string>
#include <mujoco/mujoco.h>

#include "optimization/nmpc_cost.hpp"
#include "optimization/nonlinear_model.hpp"

struct SqpConfig {
    int max_iterations = 8;
    int steps_per_control = 5;
    double gradient_epsilon = 1e-3;
    double regularization = 1e-3;
    double convergence_tolerance = 1e-5;
    double minimum_step = 1e-4;
    int max_line_search = 8;
    int qp_max_iterations = 3000;
    double qp_tolerance = 1e-6;
    double track_margin = 0.15;
    double max_cart_speed = 8.0;
    double max_joint_speed = 30.0;
    double trust_radius = 5.0;
    double max_solve_seconds = 0.0; // zero disables the time budget
    bool use_chain_derivatives = true; // Local transition derivatives + chain rule.
};

struct SqpResult {
    bool solved = false;
    bool feasible = false;
    std::vector<double> forces;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    double step_norm = 0.0; // Infinity norm of the last constrained QP step.
    int iterations = 0;
    int accepted_steps = 0;
    int line_search_steps = 0;
    double constraint_violation = 0.0;
    double solve_seconds = 0.0;
    int qp_iterations = 0;
    double qp_primal_residual = 0.0;
    double qp_stationarity_residual = 0.0;
    double qp_complementarity_residual = 0.0;
    std::string status;
};

SqpResult solve_nmpc_sqp(
    const mjModel* model,
    mjData* prediction_data,
    const State& initial_state,
    const int* qpos_addresses,
    const int* qvel_addresses,
    std::vector<double> initial_forces,
    const NmpcCostConfig& cost_config,
    const SqpConfig& config = {},
    double previous_force = 0.0
);

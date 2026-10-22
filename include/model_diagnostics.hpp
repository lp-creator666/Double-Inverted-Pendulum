#pragma once
#include "linearize.hpp"

// Checks the upright, zero-input model; uses only the independent prediction data.
void run_model_diagnostics(const mjModel* model, mjData* prediction_data,
    const int* qpos_addresses, const int* qvel_addresses,
    const Linearization& linearization);

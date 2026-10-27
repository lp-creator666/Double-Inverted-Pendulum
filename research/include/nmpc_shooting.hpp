#pragma once

#include <vector>
#include <mujoco/mujoco.h>

#include "types.hpp"

struct NmpcDefects {
    // defects[k] = candidate_states[k+1] - F(candidate_states[k], forces[k]).
    std::vector<State> defects;
    // Raw physical units are mixed here; this is a diagnostic, not a scaled merit.
    double infinity_norm = 0.0;
};

NmpcDefects compute_nmpc_dynamics_defects(
    const mjModel* model,
    mjData* prediction_data,
    const std::vector<State>& candidate_states,
    const std::vector<double>& forces,
    const int* qpos_addresses,
    const int* qvel_addresses,
    int steps_per_control
);

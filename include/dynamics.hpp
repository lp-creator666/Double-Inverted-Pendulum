#pragma once

#include <mujoco/mujoco.h>
#include "types.hpp"

State predict_state(
    const mjModel* model,
    mjData* prediction_data, //专门的mjdata
    const State& initial_state,
    const int* qpos_addresses,
    const int* qvel_addresses,
    double force,           //预测期间保持不变的力
    int steps
);

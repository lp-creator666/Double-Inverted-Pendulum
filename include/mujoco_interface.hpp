#pragma once

#include <mujoco/mujoco.h>
#include "types.hpp"

State read_state(
    const mjData* data,
    const int* qpos_addresses,
    const int* qvel_addresses
);
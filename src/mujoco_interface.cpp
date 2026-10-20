#include "mujoco_interface.hpp"

State read_state(
    const mjData* data,
    const int* qpos_addresses,
    const int* qvel_addresses
){
    State state;

    state.p = data->qpos[qpos_addresses[0]];
    state.dp = data->qvel[qvel_addresses[0]];

    state.q1 = data->qpos[qpos_addresses[1]];
    state.dq1 = data->qvel[qvel_addresses[1]];

    state.q2 = data->qpos[qpos_addresses[2]];
    state.dq2 = data->qvel[qvel_addresses[2]];

    return state;
}
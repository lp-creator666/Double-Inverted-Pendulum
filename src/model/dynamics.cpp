#include "model/dynamics.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

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

State predict_state(
    const mjModel* model,
    mjData* prediction_data,
    const State& initial_state,
    const int* qpos_addresses,
    const int* qvel_addresses,
    double force,
    int steps
) {
    if (model == nullptr || prediction_data == nullptr) {
        throw std::invalid_argument("预测模型或数据为空");
    }

    if (!initial_state.finite()) {
        throw std::invalid_argument("初始状态包含非有限值");
    }

    if (!std::isfinite(force)) {
        throw std::invalid_argument("预测输入不是有限值");
    }

    if (steps < 0) {
        throw std::invalid_argument("预测步数不能为负数");
    }

    mj_resetData(model, prediction_data);

    prediction_data->qpos[qpos_addresses[0]] = initial_state.p;
    prediction_data->qvel[qvel_addresses[0]] = initial_state.dp;

    prediction_data->qpos[qpos_addresses[1]] = initial_state.q1;
    prediction_data->qvel[qvel_addresses[1]] = initial_state.dq1;

    prediction_data->qpos[qpos_addresses[2]] = initial_state.q2;
    prediction_data->qvel[qvel_addresses[2]] = initial_state.dq2;

    mj_forward(model, prediction_data);

    const double applied_force = std::clamp(
        force,
        model->actuator_ctrlrange[0],
        model->actuator_ctrlrange[1]
    );

    for (int step = 0; step < steps; ++step) {
        mju_zero(prediction_data->ctrl, model->nu);
        prediction_data->ctrl[0] = applied_force;

        mj_step(model, prediction_data);
    }

    return read_state(
        prediction_data,
        qpos_addresses,
        qvel_addresses
    );
}

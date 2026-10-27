#pragma once

#include <vector>
#include <mujoco/mujoco.h>
#include "types.hpp"
#include <array>
#include "model/linearize.hpp"

// 非线性轨迹预测。
struct NmpcPrediction{
    // 包含初始状态，因此数量比控制力多一个。
    std::vector<State> states;

    // 相邻两个预测状态之间的时间，单位 s。
    double control_dt = 0.0;
};

NmpcPrediction predict_nonlinear_horizon(
    const mjModel* model,
    mjData* prediction_data,
    const State& initial_state,
    const int* qpos_addresses,
    const int* qvel_addresses,
    const std::vector<double>& forces,
    int steps_per_control
);

// 沿轨迹计算局部状态和输入导数。
struct NmpcSegmentLinearization {
    MatrixA A{};
    VectorB B{};
    State predicted_next{};
};

struct NmpcTrajectoryLinearization {
    std::vector<NmpcSegmentLinearization> segments;
    double state_epsilon = 0.0;
    double input_epsilon = 0.0;
};

NmpcTrajectoryLinearization linearize_nmpc_trajectory(
    const mjModel* model,
    mjData* prediction_data,
    const std::vector<State>& candidate_states,
    const std::vector<double>& forces,
    const int* qpos_addresses,
    const int* qvel_addresses,
    int steps_per_control,
    double state_epsilon = 1e-6,
    double input_epsilon = 1e-5
);

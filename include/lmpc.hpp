//预测结果和求解结果接口
#pragma once

#include <array>
#include "lmpc_config.hpp"
#include "linearize.hpp"

struct LmpcPrediction {
    int steps = 0;
    std::array<double, kStateDimension * (LmpcConfig::horizon + 1)> states{};
    std::array<double, LmpcConfig::horizon> inputs{};
    double cost = 0.0;
    bool constraints_satisfied = false;
};

struct LmpcResult {
    bool solved = false;
    bool feasible = false;
    double first_input = 0.0;
    double objective = 0.0;
    int iterations = 0;
};

LmpcPrediction predict_linear_horizon(
    const MatrixA& A, const VectorB& B,
    const StateVector& initial_state,
    const std::array<double, LmpcConfig::horizon>& inputs,
    int steps
);

bool check_lmpc_constraints(
    const LmpcPrediction& prediction,
    const LmpcConfig& config
);

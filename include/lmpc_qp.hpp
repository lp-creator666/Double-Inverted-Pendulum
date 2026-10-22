#pragma once

#include <array>
#include "lmpc_config.hpp"
#include "linearize.hpp"
#include "lqr_config.hpp"

struct LmpcQpProblem {
    static constexpr int variable_count = LmpcConfig::horizon;

    // 目标函数：
    // 0.5 * V^T H V + g^T V
    std::array<double, variable_count * variable_count> H{};
    std::array<double, variable_count> g{};

    // 输入上下界。
    std::array<double, variable_count> lower_bound{};
    std::array<double, variable_count> upper_bound{};

    double constant_cost = 0.0;
};

// As、Bs、z0、terminal_cost 均使用缩放坐标。
// H 按行存储。
LmpcQpProblem build_lmpc_qp(
    const MatrixA& As,
    const VectorB& Bs,
    const StateVector& z0,
    const LmpcConfig& config,
    const LqrConfig& weights,
    const MatrixA& terminal_cost
);
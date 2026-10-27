#pragma once

#include "model/linearize.hpp"
#include <array>
#include "control/lqr.hpp"
#include "types.hpp"

// 配置：预测时域、缩放和物理约束。
struct LmpcConfig {
    static constexpr int horizon = 20;  //统一预测时域为 20 步

    Scaling scaling{{1.0, 1.0, 0.5, 2.0, 0.5, 2.0}, 20.0};  //定义输入、轨道位置、速度约束

    // v=u/20, so these correspond to +/-20 N.
    double input_min = -1.0;
    double input_max = 1.0;

    // Bounds below use physical units.
    double cart_position_min = -3.9;
    double cart_position_max = 3.9;
    double cart_speed_min = -10.0;
    double cart_speed_max = 10.0;
    double joint_speed_min = -50.0;
    double joint_speed_max = 50.0;

    int solver_max_iterations = 100000;
    double solver_tolerance = 1e-8;
};

//预测结果和求解结果接口

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

// 二次规划：目标、输入边界与状态约束。
struct LmpcQpProblem {
    static constexpr int variable_count = LmpcConfig::horizon;

    // 目标函数：
    // 0.5 * V^T H V + g^T V
    std::array<double, variable_count * variable_count> H{};
    std::array<double, variable_count> g{};

    // 输入上下界。
    std::array<double, variable_count> lower_bound{};
    std::array<double, variable_count> upper_bound{};

    static constexpr int max_constraints = 4 * (LmpcConfig::horizon + 1);
    int constraint_count = 0;
    std::array<double, max_constraints * variable_count> constraint_matrix{};
    std::array<double, max_constraints> constraint_lower{};
    std::array<double, max_constraints> constraint_upper{};

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

// 求解：迭代参数和最优性残差。
struct LmpcQpSolverConfig {
    int max_iterations = 10000;
    double tolerance = 1e-10;
};

struct LmpcQpSolution {
    bool solved = false;
    std::array<double, LmpcQpProblem::variable_count> variables{};
    double objective = 0.0;
    double primal_residual = 0.0;
    double stationarity_residual = 0.0;
    double complementarity_residual = 0.0;
    int iterations = 0;
};

LmpcQpSolution solve_box_qp(
    const LmpcQpProblem& problem,
    const LmpcQpSolverConfig& config = {}
);

// 控制入口：从实际状态得到当前请求推力。
struct LmpcControlResult
{
    bool solved = false; //判断QP是否成功收敛
    double requested_force = 0.0; //实际请求给执行器的力
    double first_scaled_input = 0.0; //QP中的第一个输入v0
    double objective = 0.0; //当前预测问题的目标函数值
    int iterations = 0; //QP实际迭代次数
};

LmpcControlResult compute_lmpc_control(
    const MatrixA& A,
    const VectorB& B,
    const State& state,
    const LqrResult& terminal_lqr,
    const LmpcConfig& config,
    const LqrConfig& weights
);

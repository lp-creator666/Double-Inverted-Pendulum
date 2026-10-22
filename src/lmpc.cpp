#include "lmpc.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
bool finite_matrix(const MatrixA& A, const VectorB& B) {
    for (double value : A) if (!std::isfinite(value)) return false;
    for (double value : B) if (!std::isfinite(value)) return false;
    return true;
}   //检查有限值

void validate_config(const LmpcConfig& config) {
    if (config.horizon <= 0 || !std::isfinite(config.input_min) ||
        !std::isfinite(config.input_max) || config.input_min > config.input_max ||
        !std::isfinite(config.cart_position_min) || !std::isfinite(config.cart_position_max) ||
        config.cart_position_min > config.cart_position_max ||
        !std::isfinite(config.cart_speed_min) || !std::isfinite(config.cart_speed_max) ||
        config.cart_speed_min > config.cart_speed_max ||
        !std::isfinite(config.joint_speed_min) || !std::isfinite(config.joint_speed_max) ||
        config.joint_speed_min > config.joint_speed_max)
        throw std::invalid_argument("Invalid LMPC constraint configuration");
    for (double value : config.scaling.state_scale)
        if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("Invalid LMPC state scale");
    if (!std::isfinite(config.scaling.input_scale) || config.scaling.input_scale <= 0.0)
        throw std::invalid_argument("Invalid LMPC input scale");
}
}  //检查是否合法

//重点
//row为下一状态的分量，col为当前状态的分量，inputs[step]为第step步输入
LmpcPrediction predict_linear_horizon(
    const MatrixA& A, const VectorB& B,
    const StateVector& initial_state,
    const std::array<double, LmpcConfig::horizon>& inputs,
    int steps
) {
    if (steps < 0 || steps > LmpcConfig::horizon) throw std::invalid_argument("LMPC steps out of range");
    if (!finite_matrix(A, B)) throw std::invalid_argument("LMPC model contains non-finite values");
    for (double value : initial_state) if (!std::isfinite(value)) throw std::invalid_argument("Non-finite LMPC initial state");
    for (int i = 0; i < steps; ++i) if (!std::isfinite(inputs[i])) throw std::invalid_argument("Non-finite LMPC input");

    LmpcPrediction prediction;
    prediction.steps = steps;
    StateVector state = initial_state;

    for (int row = 0; row < kStateDimension; ++row)
        prediction.states[row] = state[row];
    

    //计算基础代价
    for (int step = 0; step < steps; ++step) {
        prediction.inputs[step] = inputs[step];
        StateVector next{};
        for (int row = 0; row < kStateDimension; ++row) {
            next[row] = B[row] * inputs[step];
            for (int col = 0; col < kStateDimension; ++col)
                next[row] += A[row * kStateDimension + col] * state[col];
        }
        state = next;
        for (int row = 0; row < kStateDimension; ++row)
            prediction.states[(step + 1) * kStateDimension + row] = state[row];
    }
    for (int step = 0; step < steps; ++step) {
        double state_cost = 0.0;
        for (int row = 0; row < kStateDimension; ++row) {
            const double value = prediction.states[step * kStateDimension + row];
            state_cost += value * value;
        }
        prediction.cost += state_cost + inputs[step] * inputs[step];
    }
    return prediction;
}

//检查是否满足约束
bool check_lmpc_constraints(const LmpcPrediction& prediction, const LmpcConfig& config) {
    validate_config(config);
    if (prediction.steps < 0 || prediction.steps > LmpcConfig::horizon) return false;
    for (int step = 0; step < prediction.steps; ++step) {
        if (prediction.inputs[step] < config.input_min || prediction.inputs[step] > config.input_max) return false;
    }
    for (int step = 0; step <= prediction.steps; ++step) {
        const double* z = &prediction.states[step * kStateDimension];
        const double p = z[0] * config.scaling.state_scale[0];
        const double dp = z[1] * config.scaling.state_scale[1];
        const double dq1 = z[3] * config.scaling.state_scale[3];
        const double dq2 = z[5] * config.scaling.state_scale[5];
        if (p < config.cart_position_min || p > config.cart_position_max ||
            dp < config.cart_speed_min || dp > config.cart_speed_max ||
            dq1 < config.joint_speed_min || dq1 > config.joint_speed_max ||
            dq2 < config.joint_speed_min || dq2 > config.joint_speed_max) return false;
    }
    return true;
}

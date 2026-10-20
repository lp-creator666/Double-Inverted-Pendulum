#include "linearize.hpp"

#include <cmath>
#include <stdexcept>

#include "dynamics.hpp"

namespace {
StateVector state_to_vector(const State& state) {
    return {
        state.p,
        state.dp,
        state.q1,
        state.dq1,
        state.q2,
        state.dq2
    };
}

State vector_to_state(const StateVector& values) {
    State state;
    state.p = values[0];
    state.dp = values[1];
    state.q1 = values[2];
    state.dq1 = values[3];
    state.q2 = values[4];
    state.dq2 = values[5];
    return state;
}
} // namespace

Linearization linearize_discrete(
    const mjModel* model,
    mjData* prediction_data,
    const State& equilibrium_state,
    double equilibrium_force,
    const int* qpos_addresses,
    const int* qvel_addresses,
    int prediction_steps,
    double state_epsilon,
    double input_epsilon
) {
    if (model == nullptr || prediction_data == nullptr) {
        throw std::invalid_argument("模型或预测数据为空");
    }
    if (qpos_addresses == nullptr || qvel_addresses == nullptr) {
        throw std::invalid_argument("状态地址为空");
    }
    if (!equilibrium_state.finite() || !std::isfinite(equilibrium_force)) {
        throw std::invalid_argument("平衡状态或输入不是有限值");
    }
    if (prediction_steps <= 0) {
        throw std::invalid_argument("预测步数必须为正数");
    }
    if (!std::isfinite(state_epsilon) || state_epsilon <= 0.0 ||
        !std::isfinite(input_epsilon) || input_epsilon <= 0.0) {
        throw std::invalid_argument("差分步长必须为正数且有限");
    }
    if (!std::isfinite(model->opt.timestep) || model->opt.timestep <= 0.0) {
        throw std::invalid_argument("模型时间步长必须为正数且有限");
    }

    // 验证状态在结构体和数组之间的转换是可逆的。
    const StateVector equilibrium_vector = state_to_vector(equilibrium_state);
    const State checked_state = vector_to_state(equilibrium_vector);
    if (!checked_state.finite()) {
        throw std::runtime_error("状态转换失败");
    }

    Linearization result;
    result.prediction_steps = prediction_steps;
    result.timestep = model->opt.timestep * prediction_steps;
    result.state_epsilon = state_epsilon;
    result.input_epsilon = input_epsilon;

    // A：对每个状态分量做中心差分。描述状态变化对当前状态的敏感程度
    for (int column = 0; column < kStateDimension; ++column) {
        StateVector state_plus = equilibrium_vector;
        StateVector state_minus = equilibrium_vector;
        state_plus[column] += state_epsilon;
        state_minus[column] -= state_epsilon;

        const State next_plus = predict_state(
            model, prediction_data, vector_to_state(state_plus),
            qpos_addresses, qvel_addresses, equilibrium_force, prediction_steps
        );
        const State next_minus = predict_state(
            model, prediction_data, vector_to_state(state_minus),
            qpos_addresses, qvel_addresses, equilibrium_force, prediction_steps
        );

        const StateVector next_plus_vector = state_to_vector(next_plus);
        const StateVector next_minus_vector = state_to_vector(next_minus);
        for (int row = 0; row < kStateDimension; ++row) {
            result.A[row * kStateDimension + column] =
                (next_plus_vector[row] - next_minus_vector[row]) /
                (2.0 * state_epsilon);
        }
    }

    // B：对输入做中心差分。描述状态变化对输入力的敏感程度
    const State next_plus = predict_state(
        model, prediction_data, equilibrium_state,
        qpos_addresses, qvel_addresses, equilibrium_force + input_epsilon,
        prediction_steps
    );
    const State next_minus = predict_state(
        model, prediction_data, equilibrium_state,
        qpos_addresses, qvel_addresses, equilibrium_force - input_epsilon,
        prediction_steps
    );

    const StateVector next_plus_vector = state_to_vector(next_plus);
    const StateVector next_minus_vector = state_to_vector(next_minus);
    for (int row = 0; row < kStateDimension; ++row) {
        result.B[row] =
            (next_plus_vector[row] - next_minus_vector[row]) /
            (2.0 * input_epsilon);
    }

    return result;
}

#include "nmpc_shooting.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "model/dynamics.hpp"

namespace {
State subtract(const State& lhs, const State& rhs) {
    return {
        lhs.p - rhs.p, lhs.dp - rhs.dp,
        lhs.q1 - rhs.q1, lhs.dq1 - rhs.dq1,
        lhs.q2 - rhs.q2, lhs.dq2 - rhs.dq2
    };
}

double max_abs(const State& state) {
    return std::max({std::abs(state.p), std::abs(state.dp),
                     std::abs(state.q1), std::abs(state.dq1),
                     std::abs(state.q2), std::abs(state.dq2)});
}
}

NmpcDefects compute_nmpc_dynamics_defects(
    const mjModel* model,
    mjData* prediction_data,
    const std::vector<State>& candidate_states,
    const std::vector<double>& forces,
    const int* qpos_addresses,
    const int* qvel_addresses,
    int steps_per_control
) {
    if (model == nullptr || prediction_data == nullptr) {
        throw std::invalid_argument("Prediction model or data is null");
    }
    if (qpos_addresses == nullptr || qvel_addresses == nullptr) {
        throw std::invalid_argument("Joint addresses are null");
    }
    for (int i = 0; i < 3; ++i) {
        if (qpos_addresses[i] < 0 || qpos_addresses[i] >= model->nq ||
            qvel_addresses[i] < 0 || qvel_addresses[i] >= model->nv) {
            throw std::invalid_argument("Joint address is out of range");
        }
    }
    if (steps_per_control <= 0 || !std::isfinite(model->opt.timestep) ||
        model->opt.timestep <= 0.0 || model->nu != 1 ||
        !model->actuator_ctrllimited[0]) {
        throw std::invalid_argument("Invalid prediction period or actuator");
    }
    const double lower = model->actuator_ctrlrange[0];
    const double upper = model->actuator_ctrlrange[1];
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper) {
        throw std::invalid_argument("Invalid actuator bounds");
    }
    if (candidate_states.size() != forces.size() + 1 || forces.empty()) {
        throw std::invalid_argument("Multiple-shooting trajectory sizes do not match");
    }
    for (const State& state : candidate_states) {
        if (!state.finite()) {
            throw std::invalid_argument("Candidate trajectory has a non-finite state");
        }
    }
    for (double force : forces) {
        if (!std::isfinite(force) || force < lower || force > upper) {
            throw std::invalid_argument("Candidate force is non-finite or out of range");
        }
    }
    NmpcDefects result;
    result.defects.reserve(forces.size());
    for (std::size_t k = 0; k < forces.size(); ++k) {
        const State predicted = predict_state(
            model, prediction_data, candidate_states[k],
            qpos_addresses, qvel_addresses, forces[k], steps_per_control);
        for (int i = 0; i < mjNWARNING; ++i) {
            if (prediction_data->warning[i].number != 0) {
                throw std::runtime_error("MuJoCo warning during defect evaluation");
            }
        }
        const State defect = subtract(candidate_states[k + 1], predicted);
        if (!defect.finite()) {
            throw std::runtime_error("Multiple-shooting defect is non-finite");
        }
        result.infinity_norm = std::max(result.infinity_norm, max_abs(defect));
        result.defects.push_back(defect);
    }
    return result;
}

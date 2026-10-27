#include "swingup_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

SwingupController::SwingupController(const mjModel* model, mjData* target_data,
                                     SwingupConfig config)
    : config_(config) {
    if (model == nullptr || target_data == nullptr ||
        !std::isfinite(config_.energy_gain) || config_.energy_gain < 0.0 ||
        !std::isfinite(config_.cart_damping) || config_.cart_damping < 0.0 ||
        !std::isfinite(config_.angle_damping) || config_.angle_damping < 0.0 ||
        !std::isfinite(config_.posture_gain) || config_.posture_gain < 0.0 ||
        !std::isfinite(config_.max_force) || config_.max_force <= 0.0) {
        throw std::invalid_argument("Invalid swing-up configuration");
    }
    mjData* fresh_target = mj_makeData(model);
    if (fresh_target == nullptr) throw std::runtime_error("Failed to allocate target energy data");
    mj_forward(model, fresh_target);
    mj_energyPos(model, fresh_target);
    mj_energyVel(model, fresh_target);
    target_energy_ = fresh_target->energy[0] + fresh_target->energy[1];
    mj_deleteData(fresh_target);
    if (!std::isfinite(target_energy_)) {
        throw std::runtime_error("Failed to compute target energy");
    }
}

double SwingupController::compute_force(const mjModel* model, mjData* data,
                                        const State& state) const {
    if (model == nullptr || data == nullptr || !state.finite()) {
        throw std::invalid_argument("Invalid swing-up state");
    }
    mj_energyPos(model, data);
    mj_energyVel(model, data);
    const double energy = data->energy[0] + data->energy[1];
    if (!std::isfinite(energy)) throw std::runtime_error("Non-finite swing-up energy");
    const double error = target_energy_ - energy;
    const double absolute_q2 = state.q1 + state.q2;
    const double absolute_dq2 = state.dq1 + state.dq2;
    double phase = std::sin(state.q1) * state.dq1 +
                   0.5 * std::sin(absolute_q2) * absolute_dq2;
    if (std::abs(phase) < 1e-6) phase = state.q1 + 0.5 * absolute_q2;
    const double pump = config_.energy_gain * error * (phase >= 0.0 ? 1.0 : -1.0);
    const double damping = config_.cart_damping * state.dp +
                           config_.angle_damping * (state.dq1 + absolute_dq2);
    const double posture = config_.posture_gain *
                           (std::sin(state.q1) + 0.5 * std::sin(absolute_q2));
    return std::clamp(pump - damping - posture,
                      -config_.max_force, config_.max_force);
}

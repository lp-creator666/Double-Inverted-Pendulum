#include "optimization/nmpc_cost.hpp"

#include <cmath>
#include <stdexcept>

namespace {
// Equals 2*(1-cos(angle)), without cancellation near zero.
double periodic_angle_cost(double angle) {
    const double s = std::sin(0.5 * angle);
    return 4.0 * s * s;
}
}

double compute_nmpc_state_cost(const State& state, const NmpcCostConfig& config) {
    if (!state.finite()) {
        throw std::invalid_argument("NMPC state is not finite");
    }
    for (double weight : config.state_weights) {
        if (!std::isfinite(weight) || weight < 0.0) {
            throw std::invalid_argument("Invalid NMPC state weight");
        }
    }

    const double angle_2 = state.q1 + state.q2;
    const double speed_2 = state.dq1 + state.dq2;
    if (!std::isfinite(angle_2) || !std::isfinite(speed_2)) {
        throw std::runtime_error("NMPC absolute state overflow");
    }
    const auto& w = config.state_weights;
    const double cost =
        w[0] * state.p * state.p +
        w[1] * state.dp * state.dp +
        w[2] * periodic_angle_cost(state.q1) +
        w[3] * state.dq1 * state.dq1 +
        w[4] * periodic_angle_cost(angle_2) +
        w[5] * speed_2 * speed_2;
    if (!std::isfinite(cost)) {
        throw std::runtime_error("NMPC state cost is not finite");
    }
    return cost;
}

double compute_nmpc_trajectory_cost(
    const NmpcPrediction& prediction,
    const std::vector<double>& forces,
    const NmpcCostConfig& config,
    double previous_force
) {
    if (prediction.states.size() != forces.size() + 1 || forces.empty()) {
        throw std::invalid_argument("NMPC trajectory and input sizes do not match");
    }
    if (!std::isfinite(previous_force) ||
        !std::isfinite(config.input_weight) || config.input_weight < 0.0 ||
        !std::isfinite(config.input_change_weight) || config.input_change_weight < 0.0 ||
        !std::isfinite(config.terminal_weight) || config.terminal_weight < 0.0) {
        throw std::invalid_argument("Invalid NMPC trajectory cost settings");
    }

    double total = 0.0;
    for (std::size_t k = 0; k < forces.size(); ++k) {
        const double state_cost = compute_nmpc_state_cost(prediction.states[k], config);
        const double force = forces[k];
        if (!std::isfinite(force)) {
            throw std::invalid_argument("NMPC force is not finite");
        }
        total += state_cost + config.input_weight * force * force;
        const double delta = force - (k == 0 ? previous_force : forces[k - 1]);
        total += config.input_change_weight * delta * delta;
        if (!std::isfinite(total)) {
            throw std::runtime_error("NMPC trajectory cost is not finite");
        }
    }
    total += config.terminal_weight *
             compute_nmpc_state_cost(prediction.states.back(), config);
    if (!std::isfinite(total)) {
        throw std::runtime_error("NMPC trajectory cost is not finite");
    }
    return total;
}

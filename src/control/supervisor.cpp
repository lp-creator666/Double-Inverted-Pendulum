#include "control/supervisor.hpp"

#include <cmath>
#include <stdexcept>

State local_balance_state(const State& state) {
    if (!state.finite() || !std::isfinite(state.q1+state.q2))
        throw std::invalid_argument("Non-finite balance state");
    const double two_pi=2.0*std::acos(-1.0);
    State local=state;
    local.q1=std::remainder(state.q1,two_pi);
    local.q2=std::remainder(state.q1+state.q2,two_pi)-local.q1;
    return local;
}

Supervisor::Supervisor(SupervisorConfig config) : config_(config) {
    if (!std::isfinite(config_.enter_angle) || config_.enter_angle <= 0.0 ||
        !std::isfinite(config_.exit_angle) || config_.exit_angle < config_.enter_angle ||
        !std::isfinite(config_.enter_speed) || config_.enter_speed <= 0.0 ||
        !std::isfinite(config_.exit_speed) || config_.exit_speed < config_.enter_speed ||
        !std::isfinite(config_.enter_position) || config_.enter_position <= 0.0 ||
        !std::isfinite(config_.exit_position) || config_.exit_position < config_.enter_position) {
        throw std::invalid_argument("Invalid supervisor thresholds");
    }
}

bool Supervisor::in_capture_region(const State& state, bool use_enter_threshold) const {
    if (!state.finite()) return false;
    const double angle = use_enter_threshold ? config_.enter_angle : config_.exit_angle;
    const double speed = use_enter_threshold ? config_.enter_speed : config_.exit_speed;
    const double position = use_enter_threshold ? config_.enter_position : config_.exit_position;
    if (!std::isfinite(state.q1+state.q2) || !std::isfinite(state.dq1+state.dq2)) return false;
    const State local=local_balance_state(state);
    const double absolute_q2 = local.q1 + local.q2;
    const double absolute_dq2 = state.dq1 + state.dq2;
    return std::abs(local.q1) <= angle && std::abs(absolute_q2) <= angle &&
           std::abs(state.dq1) <= speed && std::abs(absolute_dq2) <= speed &&
           std::abs(state.p) <= position && std::abs(state.dp) <= speed;
}

ControlStage Supervisor::update(const State& state, bool sqp_available) {
    if (!state.finite()) {
        stage_ = ControlStage::Fault;
        return stage_;
    }
    if (stage_ == ControlStage::Fault) return stage_;
    if (stage_ == ControlStage::SwingUp) {
        if (in_capture_region(state, true)) stage_ = ControlStage::Lqr;
    } else if (stage_ == ControlStage::Sqp) {
        if (in_capture_region(state, true)) stage_ = ControlStage::Lqr;
        else if (!sqp_available) stage_ = ControlStage::Fault;
    } else if (stage_ == ControlStage::Lqr) {
        if (!in_capture_region(state, false)) {
            stage_ = sqp_available ? ControlStage::Sqp : ControlStage::Fault;
        }
    }
    return stage_;
}

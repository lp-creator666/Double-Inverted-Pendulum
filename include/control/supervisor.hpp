#pragma once

#include "types.hpp"

enum class ControlStage {
    SwingUp,
    Sqp,
    Lqr,
    Fault
};

// Preserve the two absolute orientations modulo 2*pi for local LQR feedback.
State local_balance_state(const State& state);

struct SupervisorConfig {
    double enter_angle = 0.12;
    double exit_angle = 0.18;
    double enter_speed = 1.0;
    double exit_speed = 1.8;
    double enter_position = 0.6;
    double exit_position = 1.0;
};

class Supervisor {
public:
    explicit Supervisor(SupervisorConfig config = {});

    ControlStage update(const State& state, bool sqp_available);
    void start_swingup() { stage_ = ControlStage::SwingUp; }
    ControlStage stage() const { return stage_; }
    bool in_capture_region(const State& state, bool use_enter_threshold) const;

private:
    SupervisorConfig config_;
    ControlStage stage_ = ControlStage::Sqp;
};

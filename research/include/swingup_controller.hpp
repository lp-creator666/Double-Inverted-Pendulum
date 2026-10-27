#pragma once

#include <mujoco/mujoco.h>

#include "types.hpp"

struct SwingupConfig {
    double energy_gain = 2.0;
    double cart_damping = 1.0;
    double angle_damping = 0.2;
    double posture_gain = 2.0;
    double max_force = 20.0;
};

class SwingupController {
public:
    SwingupController(const mjModel* model, mjData* target_data,
                      SwingupConfig config = {});

    double compute_force(const mjModel* model, mjData* data,
                         const State& state) const;
    double target_energy() const { return target_energy_; }

private:
    SwingupConfig config_;
    double target_energy_ = 0.0;
};

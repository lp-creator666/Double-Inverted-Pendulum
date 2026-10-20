#pragma once

#include <array>

#include <mujoco/mujoco.h>

#include "types.hpp"

constexpr int kStateDimension = 6;

using StateVector = std::array<double, kStateDimension>;
using MatrixA = std::array<double, kStateDimension * kStateDimension>;
using VectorB = std::array<double, kStateDimension>;

struct Linearization {
    MatrixA A{};
    VectorB B{};

    double timestep = 0.0;
    int prediction_steps = 0;
    double state_epsilon = 0.0;
    double input_epsilon = 0.0;
};

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
);